// Real Subchannel Q for the PSX core ("real-subq"), see psx_subq.h
//
// Sources, in order of priority:
//   1. physical disc: raw P-W read together with the sector data (read-ahead ring, no extra drive access)
//   2. CHD with subcode (RW / RW_RAW)
//   3. CloneCD style .sub next to the .cue (96 bytes per sector, whole disc)
//   4. .sbi file: the modified (bad CRC) Q frames of LibCrypt discs
// Frames without a source are reported as "not present" and the core synthesizes them as before.

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>

#include "../../file_io.h"
#include "../../user_io.h"
#include "../../spi.h"
#include "../../cd.h"
#include "../chd/mister_chd.h"
#include "../physical_disc/physical_disc.h"
#include "psx_subq.h"

#define PREGAP_FRAMES 150

typedef struct
{
	int     frame;
	uint8_t q[10];
} sbi_entry_t;

typedef struct
{
	uint8_t adr;
	uint8_t point;
	uint8_t q[12];
} leadin_entry_t;

static toc_t          *s_toc = NULL;
static int             s_enabled = 0;
static int             s_phys = 0;

static fileTYPE        s_sub = {};
static int             s_sub_open = 0;
static int             s_sub_frames = 0;

static uint8_t        *s_chd_hunk = NULL;
static int             s_chd_hunknum = -1;

static sbi_entry_t     s_sbi[128];
static int             s_nsbi = 0;

static leadin_entry_t  s_leadin[110];
static int             s_nleadin = 0;

// drive Q phase correction (physical disc): Q for LBA x is found at ring[x - s_q_offset]
static int             s_q_offset = 0;
static int             s_off_candidate = 0;
static int             s_off_votes = 0;

static uint8_t         s_last_phys_seq = 0;
static uint8_t         s_last_getq_seq = 0;

static uint32_t        s_stat_real = 0, s_stat_bad = 0, s_stat_none = 0;

// ------------------------------------------------------------------------------------------

static uint8_t bcd(int v) { return (uint8_t)(((v / 10) << 4) | (v % 10)); }

static int unbcd(uint8_t v)
{
	int hi = v >> 4, lo = v & 15;
	if (hi > 9 || lo > 9) return -1;
	return hi * 10 + lo;
}

static uint16_t q_crc(const uint8_t *q)
{
	uint16_t crc = 0;
	for (int i = 0; i < 10; i++)
	{
		crc ^= (uint16_t)q[i] << 8;
		for (int b = 0; b < 8; b++) crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
	}
	return (uint16_t)~crc;
}

static int q_crc_ok(const uint8_t *q)
{
	return q_crc(q) == (uint16_t)((q[10] << 8) | q[11]);
}

// absolute frame from Q bytes 7..9, -1 if not valid BCD
static int q_abs_frame(const uint8_t *q)
{
	int m = unbcd(q[7]), s = unbcd(q[8]), f = unbcd(q[9]);
	if (m < 0 || s < 0 || f < 0 || s > 59 || f > 74) return -1;
	return (m * 60 + s) * 75 + f;
}

// raw interleaved P-W (as returned by READ CD sub-channel 001b): Q is bit 6 of each byte
static void q_from_raw(const uint8_t *sub, uint8_t *q)
{
	for (int b = 0; b < 12; b++)
	{
		uint8_t v = 0;
		for (int bit = 0; bit < 8; bit++) v = (uint8_t)((v << 1) | ((sub[b * 8 + bit] >> 6) & 1));
		q[b] = v;
	}
}

// deinterleaved P-W (CloneCD .sub, 12 bytes per channel): Q = bytes 12..23
static void q_from_cooked(const uint8_t *sub, uint8_t *q)
{
	memcpy(q, sub + 12, 12);
}

// picks the interpretation with a valid CRC, falls back to the preferred one
static void q_from_sub(const uint8_t *sub, uint8_t *q, int prefer_raw)
{
	uint8_t a[12], b[12];
	q_from_raw(sub, a);
	q_from_cooked(sub, b);
	if (q_crc_ok(a) && !q_crc_ok(b)) memcpy(q, a, 12);
	else if (q_crc_ok(b) && !q_crc_ok(a)) memcpy(q, b, 12);
	else memcpy(q, prefer_raw ? a : b, 12);
}

static int st_of(const uint8_t *q)
{
	return PSX_SUBQ_ST_PRESENT | (q_crc_ok(q) ? PSX_SUBQ_ST_CRCOK : 0);
}

// ------------------------------------------------------------------------------------------
// physical disc

static int phys_q_at(int lba, uint8_t *q)
{
	uint8_t sub[96];
	if (!physical_disc_peek_sub(lba, sub)) return 0;
	q_from_raw(sub, q);
	return 1;
}

static void learn_offset(int d)
{
	if (d < -3 || d > 3) return;
	if (d == s_off_candidate) s_off_votes++;
	else
	{
		s_off_candidate = d;
		s_off_votes = 1;
	}

	if (s_off_candidate && s_off_votes >= 16)
	{
		s_q_offset += s_off_candidate;
		printf("PSX subq: drive delivers Q shifted by %d frame(s), correcting (offset now %d)\n", s_off_candidate, s_q_offset);
		s_off_candidate = 0;
		s_off_votes = 0;
	}
}

static int get_q_phys(int frame, uint8_t *q)
{
	int src = frame - PREGAP_FRAMES - s_q_offset;
	if (!phys_q_at(src, q)) return 0;

	if (q_crc_ok(q))
	{
		if ((q[0] & 3) != 1) return st_of(q); // ADR 2/3 (catalog/ISRC): passed on, the core ignores it like the real controller

		int f = q_abs_frame(q);
		if (f == frame)
		{
			learn_offset(0);
			return st_of(q);
		}

		// drive returned the Q of another frame: do not pass it, the core synthesizes this one
		if (f >= 0) learn_offset(f - frame);
		return 0;
	}

	// bad CRC (LibCrypt or real read error). Only trust it if both neighbours are aligned,
	// otherwise a misplaced frame could fake a protection bit.
	uint8_t n[12];
	if (!phys_q_at(src - 1, n) || !q_crc_ok(n) || q_abs_frame(n) != frame - 1) return 0;
	if (!phys_q_at(src + 1, n) || !q_crc_ok(n) || q_abs_frame(n) != frame + 1) return 0;
	return PSX_SUBQ_ST_PRESENT;
}

// ------------------------------------------------------------------------------------------
// images

static int track_of_frame(int frame)
{
	if (!s_toc) return -1;
	for (int i = 0; i < s_toc->last; i++)
	{
		if (frame >= s_toc->tracks[i].start && frame <= s_toc->tracks[i].end) return i;
	}
	return -1;
}

static int get_q_chd(int frame, uint8_t *q)
{
	int i = track_of_frame(frame);
	if (i < 0 || !s_toc->chd_f || !s_chd_hunk || s_toc->tracks[i].sbc_type == SUBCODE_NONE) return 0;

	// same mapping as psx_read_cd(): fake pregap sectors are not in the file
	if (s_toc->tracks[i + 1].pregap && frame > (s_toc->tracks[i + 1].start - s_toc->tracks[i + 1].indexes[1])) return 0;

	uint8_t sub[96];
	int lba = frame - s_toc->tracks[0].indexes[1] + s_toc->tracks[i].offset;
	if (mister_chd_read_sector(s_toc->chd_f, lba, 0, 2352, 96, sub, s_chd_hunk, &s_chd_hunknum) != CHDERR_NONE) return 0;

	q_from_sub(sub, q, s_toc->tracks[i].sbc_type == SUBCODE_RW_RAW);
	return st_of(q);
}

static int get_q_subfile(int frame, uint8_t *q)
{
	int lba = frame - PREGAP_FRAMES;
	if (!s_sub_open || lba < 0 || lba >= s_sub_frames) return 0;

	uint8_t sub[96];
	if (!FileSeek(&s_sub, (__off64_t)lba * 96, SEEK_SET)) return 0;
	if (FileReadAdv(&s_sub, sub, 96) != 96) return 0;

	q_from_sub(sub, q, 0);
	return st_of(q);
}

static int get_q_sbi(int frame, uint8_t *q)
{
	for (int i = 0; i < s_nsbi; i++)
	{
		if (s_sbi[i].frame == frame)
		{
			memcpy(q, s_sbi[i].q, 10);
			uint16_t crc = q_crc(q) ^ 0x8001; // SBI holds the modified Q without CRC: these frames have a bad CRC on the disc
			q[10] = crc >> 8;
			q[11] = crc & 0xFF;
			return PSX_SUBQ_ST_PRESENT;
		}
	}
	return 0;
}

int psx_subq_get(int frame, uint8_t *q)
{
	memset(q, 0, 12);
	if (!s_enabled) return 0;

	int st = 0;
	if (s_phys) st = get_q_phys(frame, q);
	else if (s_toc && s_toc->chd_f) st = get_q_chd(frame, q);
	else if (s_sub_open) st = get_q_subfile(frame, q);

	if (!st) st = get_q_sbi(frame, q);
	if (!st) memset(q, 0, 12);
	return st;
}

// ------------------------------------------------------------------------------------------
// lead-in (GetQ)

static void leadin_add(uint8_t adr_ctrl_q0, uint8_t point, uint8_t amin, uint8_t asec, uint8_t afrm, uint8_t pmin, uint8_t psec, uint8_t pfrm)
{
	if (s_nleadin >= (int)(sizeof(s_leadin) / sizeof(s_leadin[0]))) return;
	leadin_entry_t *e = &s_leadin[s_nleadin++];
	e->adr = adr_ctrl_q0 & 0x0F;
	e->point = point;
	uint8_t *q = e->q;
	q[0] = adr_ctrl_q0; q[1] = 0x00; q[2] = point;
	q[3] = amin; q[4] = asec; q[5] = afrm; q[6] = 0x00;
	q[7] = pmin; q[8] = psec; q[9] = pfrm;
	uint16_t crc = q_crc(q);
	q[10] = crc >> 8;
	q[11] = crc & 0xFF;
}

static void leadin_from_toc(void)
{
	s_nleadin = 0;
	if (!s_toc || !s_toc->last) return;

	for (int i = 0; i < s_toc->last; i++)
	{
		int f = (i ? s_toc->tracks[i].start : 0) + s_toc->tracks[i].indexes[1]; // index 01, same as GetTD in send_cue_and_metadata()
		uint8_t q0 = s_toc->tracks[i].type ? 0x41 : 0x01;
		leadin_add(q0, bcd(i + 1), 0, 0, 0, bcd(f / 4500), bcd((f / 75) % 60), bcd(f % 75));
	}

	uint8_t first = s_toc->tracks[0].type ? 0x41 : 0x01;
	uint8_t last = s_toc->tracks[s_toc->last - 1].type ? 0x41 : 0x01;
	int lo = s_toc->end;
	leadin_add(first, 0xA0, 0, 0, 0, 0x01, s_toc->tracks[0].type ? 0x20 : 0x00, 0x00); // first track, disc type (20h = CD-ROM XA)
	leadin_add(last,  0xA1, 0, 0, 0, bcd(s_toc->last), 0x00, 0x00);                      // last track
	leadin_add(last,  0xA2, 0, 0, 0, bcd(lo / 4500), bcd((lo / 75) % 60), bcd(lo % 75));  // lead-out
}

static int leadin_from_drive(void)
{
	uint8_t buf[2048];
	int len = physical_disc_read_full_toc(buf, sizeof(buf));
	if (len < 4 + 11) return 0;

	int n = (len - 4) / 11;

	// MMC drives normally return MSF/POINT of the full TOC in binary, some pass the BCD values
	// of the disc through. Decide with the lead-out (A2) against the TOC.
	int binary = 1;
	for (int i = 0; i < n; i++)
	{
		uint8_t *d = buf + 4 + i * 11;
		if (d[0] == 1 && d[3] == 0xA2)
		{
			int fb = (d[8] * 60 + d[9]) * 75 + d[10];
			int fd = (unbcd(d[8]) * 60 + unbcd(d[9])) * 75 + unbcd(d[10]);
			if (s_toc && fd == s_toc->end && fb != s_toc->end) binary = 0;
		}
	}

	s_nleadin = 0;
	for (int i = 0; i < n; i++)
	{
		uint8_t *d = buf + 4 + i * 11;
		if (d[0] != 1) continue;                                // PSX: first session only
		uint8_t q0 = (uint8_t)(((d[1] & 0x0F) << 4) | (d[1] >> 4)); // MMC: ADR in high nibble, Q: CONTROL in high nibble
		uint8_t point = d[3];
		uint8_t m = d[4], s = d[5], f = d[6], pm = d[8], ps = d[9], pf = d[10];
		if (binary)
		{
			if (point < 0xA0) point = bcd(point);
			m = bcd(m); s = bcd(s); f = bcd(f);
			pm = bcd(pm);                                      // A0/A1: PMIN = first/last track number
			if (d[3] != 0xA0 && d[3] != 0xA1) { ps = bcd(ps); pf = bcd(pf); } // A0: PSEC = disc type, stays as is
		}
		leadin_add(q0, point, m, s, f, pm, ps, pf);
	}
	printf("PSX subq: lead-in from drive, %d entries (%s)\n", s_nleadin, binary ? "binary" : "bcd");
	return s_nleadin > 0;
}

static int leadin_find(uint8_t adr, uint8_t point)
{
	for (int i = 0; i < s_nleadin; i++)
	{
		// psx-spx: the controller only compares the lower 2 bits of ADR (ADR 5 == ADR 1)
		if ((s_leadin[i].adr & 3) == (adr & 3) && s_leadin[i].point == point) return i;
	}
	return -1;
}

// ------------------------------------------------------------------------------------------

static void load_sbi(fileTYPE *f)
{
	s_nsbi = 0;
	if (!f || !f->size) return;

	uint8_t hdr[4];
	FileSeek(f, 0, SEEK_SET);
	if (FileReadAdv(f, hdr, 4) != 4 || memcmp(hdr, "SBI", 3)) return;

	uint8_t e[4];
	while (s_nsbi < (int)(sizeof(s_sbi) / sizeof(s_sbi[0])) && FileReadAdv(f, e, 4) == 4)
	{
		int len = (e[3] == 2 || e[3] == 3) ? 3 : 10;
		uint8_t d[10] = {};
		if (FileReadAdv(f, d, len) != len) break;
		int m = unbcd(e[0]), s = unbcd(e[1]), fr = unbcd(e[2]);
		if (m < 0 || s < 0 || fr < 0 || len != 10) continue;
		s_sbi[s_nsbi].frame = (m * 60 + s) * 75 + fr;
		memcpy(s_sbi[s_nsbi].q, d, 10);
		s_nsbi++;
	}
	printf("PSX subq: %d SBI frames loaded\n", s_nsbi);
}

void psx_subq_reset(void)
{
	if (s_sub_open) FileClose(&s_sub);
	s_sub_open = 0;
	s_sub_frames = 0;
	if (s_chd_hunk) free(s_chd_hunk);
	s_chd_hunk = NULL;
	s_chd_hunknum = -1;
	s_toc = NULL;
	s_enabled = 0;
	s_phys = 0;
	s_nsbi = 0;
	s_nleadin = 0;
	s_q_offset = 0;
	s_off_candidate = 0;
	s_off_votes = 0;
	s_stat_real = s_stat_bad = s_stat_none = 0;
}

void psx_subq_setup(toc_t *toc, int phys, const char *image, fileTYPE *sbi)
{
	psx_subq_reset();
	if (!toc || !toc->last) return;

	s_toc = toc;
	s_phys = phys;
	s_enabled = 1; // always on: frames without a source are synthesized by the core, GetQ gets a lead-in

	const char *src = "none (core synthesizes, GetQ from TOC)";

	if (phys)
	{
		src = physical_disc_subq_capable() ? "physical disc raw P-W" : "none: drive returns no subchannel";
	}
	else if (toc->chd_f)
	{
		int has_sub = 0;
		for (int i = 0; i < toc->last; i++) if (toc->tracks[i].sbc_type != SUBCODE_NONE) has_sub = 1;
		if (has_sub)
		{
			s_chd_hunk = (uint8_t *)malloc(toc->chd_hunksize);
			s_chd_hunknum = -1;
			if (s_chd_hunk) src = "CHD subcode";
		}
	}
	else if (image)
	{
		// CloneCD style subchannel file next to the cue: <name>.sub
		char path[1024];
		snprintf(path, sizeof(path), "%s", image);
		char *ext = strrchr(path, '.');
		if (ext && (size_t)(ext - path) + 4 < sizeof(path))
		{
			strcpy(ext, ".sub");
			if (FileOpen(&s_sub, path, 1))
			{
				s_sub_frames = (int)(s_sub.size / 96);
				if (s_sub_frames && s_sub.size % 96 == 0)
				{
					s_sub_open = 1;
					src = ".sub file";
				}
				else FileClose(&s_sub);
			}
		}
	}

	load_sbi(sbi);

	if (!(phys && leadin_from_drive())) leadin_from_toc();

	printf("PSX subq: real Subchannel Q enabled, source: %s%s, lead-in entries %d\n", src, s_nsbi ? " + SBI" : "", s_nleadin);
}

int psx_subq_enabled(void)
{
	return s_enabled;
}

static void send_q(uint32_t tag, uint8_t status, const uint8_t *q)
{
	spi_uio_cmd_cont(UIO_CD_SET);
	spi_w(tag & 0xFFFF);
	spi_w((uint16_t)((status << 8) | ((tag >> 16) & 0xFF)));
	for (int i = 0; i < 6; i++) spi_w((uint16_t)(q[i * 2] | (q[i * 2 + 1] << 8)));
	DisableIO();
}

void psx_subq_on_sector(uint32_t lba)
{
	if (!s_enabled) return;

	// the core shows the Q two frames ahead of the sector data (see cd_top.vhd, subchannelSector = lastReadSector + 2)
	uint32_t frame = lba + 2;
	uint8_t q[12];
	int st = psx_subq_get(frame, q);

	if (!st) s_stat_none++;
	else if (st & PSX_SUBQ_ST_CRCOK) s_stat_real++;
	else if (++s_stat_bad <= 64)
	{
		printf("PSX subq: frame %02d:%02d:%02d has bad CRC (LibCrypt/read error), core keeps last position\n",
			frame / 4500, (frame / 75) % 60, frame % 75);
	}

	send_q(frame, (uint8_t)st, q);
}

void psx_subq_poll(void)
{
	uint16_t seq = spi_uio_cmd_cont(UIO_CD_GET); // also keeps the core heartbeat running
	uint16_t tag_lo = spi_w(0);
	uint16_t tag_hi = spi_w(0);
	uint16_t getq = spi_w(0);
	DisableIO();

	if (!s_enabled) return;

	uint8_t phys_seq = seq & 0xFF;
	uint8_t getq_seq = seq >> 8;

	if (phys_seq != s_last_phys_seq)
	{
		// GetLocP while the drive is not reading: Q of the current physical position
		s_last_phys_seq = phys_seq;
		uint32_t frame = tag_lo | ((uint32_t)(tag_hi & 0xFF) << 16);
		uint8_t q[12];
		int st = psx_subq_get(frame, q);
		send_q(frame, (uint8_t)st, q);
	}

	if (getq_seq != s_last_getq_seq)
	{
		s_last_getq_seq = getq_seq;
		uint8_t adr = getq >> 8, point = getq & 0xFF;
		int i = leadin_find(adr, point);
		uint8_t q[12] = {};
		uint8_t st = PSX_SUBQ_ST_LEADIN;
		if (i >= 0)
		{
			memcpy(q, s_leadin[i].q, 12);
			st |= PSX_SUBQ_ST_PRESENT | PSX_SUBQ_ST_CRCOK;
		}
		else st |= PSX_SUBQ_ST_NOTFOUND;
		printf("PSX subq: GetQ adr=%X point=%02X -> %s\n", adr, point, i >= 0 ? "found" : "not found");
		send_q(getq, st, q);
	}
}

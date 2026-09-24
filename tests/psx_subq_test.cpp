// Host unit test for support/psx/psx_subq.cpp (not part of the MiSTer build)
// build: g++ -O1 -I.. -I../lib/libchdr/include -o psx_subq_test psx_subq_test.cpp && ./psx_subq_test
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <vector>

#define SPI_H // replace spi.h with the stubs below
#include "../file_io.h"
uint16_t spi_uio_cmd_cont(uint16_t cmd);
uint16_t spi_w(uint16_t w);
void DisableIO();

// ---------------------------------------------------------------- stubs
static std::vector<uint16_t> g_words;          // words of the current SPI transaction
static std::vector<std::vector<uint16_t>> g_tx; // finished transactions
static uint16_t g_cdget_resp[4];
uint16_t spi_uio_cmd_cont(uint16_t cmd) { g_words.clear(); g_words.push_back(cmd); return cmd == 0x34 ? g_cdget_resp[0] : 0; }
uint16_t spi_w(uint16_t w) { g_words.push_back(w); size_t n = g_words.size() - 1; return (g_words[0] == 0x34 && n < 4) ? g_cdget_resp[n] : 0; }
void DisableIO() { g_tx.push_back(g_words); g_words.clear(); }
int FileOpen(fileTYPE *, const char *, char) { return 0; }
void FileClose(fileTYPE *) {}
fileTYPE::fileTYPE() { memset((void *)this, 0, sizeof(*this)); }
fileTYPE::~fileTYPE() {}
int FileSeek(fileTYPE *, __off64_t, int) { return 0; }
static const uint8_t *g_sbi = NULL; static int g_sbi_len = 0, g_sbi_pos = 0;
int FileReadAdv(fileTYPE *, void *b, int len, int) { int n = g_sbi_len - g_sbi_pos; if (n > len) n = len; if (n < 0) n = 0; memcpy(b, g_sbi + g_sbi_pos, n); g_sbi_pos += n; return n; }
#include "../support/chd/mister_chd.h"
chd_error mister_chd_read_sector(chd_file *, int, uint32_t, uint32_t, int, uint8_t *, uint8_t *, int *) { return CHDERR_READ_ERROR; }

// physical disc model: ring holds raw P-W for [ring_lo, ring_hi), Q of lba x stored at x - drive_offset
static int ring_lo = 0, ring_hi = 0, drive_offset = 0;
static uint8_t tbcd(int v) { return (uint8_t)(((v / 10) << 4) | (v % 10)); }
static uint16_t crc(const uint8_t *q) { uint16_t c = 0; for (int i = 0; i < 10; i++) { c ^= (uint16_t)q[i] << 8; for (int b = 0; b < 8; b++) c = (c & 0x8000) ? (uint16_t)((c << 1) ^ 0x1021) : (uint16_t)(c << 1); } return (uint16_t)~c; }
static bool is_libcrypt(int frame) { return frame == 14105 || frame == 14110; }
static void model_q(int lba, uint8_t *q)
{
	int f = lba + 150, r = lba;
	if (is_libcrypt(f)) f ^= 0x0404;
	uint8_t t[12] = { 0x41, 0x01, 0x01, tbcd(r / 4500), tbcd((r / 75) % 60), tbcd(r % 75), 0, tbcd(f / 4500), tbcd((f / 75) % 60), tbcd(f % 75) };
	uint16_t c = crc(t);
	if (is_libcrypt(lba + 150)) c ^= 0x8001;
	t[10] = c >> 8; t[11] = c & 0xFF;
	memcpy(q, t, 12);
}
int physical_disc_peek_sub(int lba, uint8_t *sub)
{
	if (lba < ring_lo || lba >= ring_hi) return 0;
	uint8_t q[12];
	model_q(lba + drive_offset, q);
	memset(sub, 0, 96);
	for (int b = 0; b < 12; b++) for (int bit = 0; bit < 8; bit++) if (q[b] & (0x80 >> bit)) sub[b * 8 + bit] |= 0x40;
	for (int i = 0; i < 96; i++) sub[i] |= 0x80; // P channel noise
	return 1;
}
int physical_disc_subq_capable(void) { return 1; }
int physical_disc_read_full_toc(uint8_t *d, int maxlen)
{
	// binary MSF, ADR in high nibble (MMC). track 1 at 00:02:00, lead-out at 60:00:00
	uint8_t e[3][11] = { { 1, 0x14, 0, 0x01, 0, 0, 0, 0, 0, 2, 0 }, { 1, 0x14, 0, 0xA0, 0, 0, 0, 0, 1, 0x20, 0 }, { 1, 0x14, 0, 0xA2, 0, 0, 0, 0, 60, 0, 0 } };
	int len = 4 + 33; if (len > maxlen) return -1;
	d[0] = 0; d[1] = len - 2; d[2] = 1; d[3] = 1; memcpy(d + 4, e, 33); return len;
}

#include "../support/psx/psx_subq.cpp"

// ---------------------------------------------------------------- tests
static int fails = 0;
#define CHECK(c, msg) do { if (c) printf("ok   %s\n", msg); else { printf("FAIL %s\n", msg); fails++; } } while (0)

int main()
{
	static toc_t toc;
	
	toc.last = 1; toc.end = 60 * 60 * 75;
	toc.tracks[0].start = 150; toc.tracks[0].end = toc.end - 1; toc.tracks[0].type = TT_MODE2; toc.tracks[0].indexes[1] = 150;

	// SBI with one LibCrypt frame at 09:20:45 (backup copy, not in the core's 16 bit table)
	uint8_t sbi[4 + 14] = { 'S', 'B', 'I', 0, 0x09, 0x20, 0x45, 0x01, 0x41, 0x01, 0x01, 0x09, 0x18, 0x45, 0x00, 0x09, 0x24, 0x45 };
	g_sbi = sbi; g_sbi_len = sizeof(sbi); g_sbi_pos = 0;
	fileTYPE f = {}; f.size = sizeof(sbi);

	psx_subq_setup(&toc, 1, NULL, &f);
	CHECK(psx_subq_enabled(), "enabled");
	CHECK(s_nleadin == 3, "lead-in from drive: 3 entries");
	int a2 = leadin_find(1, 0xA2);
	CHECK(a2 >= 0 && s_leadin[a2].q[0] == 0x41 && s_leadin[a2].q[7] == 0x60 && s_leadin[a2].q[8] == 0x00, "A2 converted to BCD 60:00:00, control/ADR swapped to 41h");
	int t1 = leadin_find(1, 0x01);
	CHECK(t1 >= 0 && s_leadin[t1].q[8] == 0x02, "track 01 at 00:02:00");

	uint8_t q[12]; int st;
	ring_lo = 13900; ring_hi = 14300;
	st = psx_subq_get(14100, q);
	CHECK(st == 3 && q[9] == tbcd(14100 % 75), "normal frame: present+crc ok, MSF matches");
	st = psx_subq_get(14105, q);
	CHECK(st == 1, "LibCrypt frame: present, bad CRC (neighbours aligned)");
	st = psx_subq_get(20000, q);
	CHECK(st == 0, "frame outside ring: not present (core synthesizes)");
	st = psx_subq_get(42045, q);
	CHECK(st == 1 && q[3] == 0x09 && q[4] == 0x18 && q[8] == 0x24, "outside ring but in SBI: SBI Q with bad CRC");

	// ring edge: bad frame whose neighbour is not in the ring must not be trusted
	ring_lo = 13956; // lba 13955 (frame 14105) is outside, test 14110 with lba 13959 neighbour missing
	ring_lo = 13960; // 14110 -> lba 13960, neighbour 13959 missing
	st = psx_subq_get(14110, q);
	CHECK(st == 0, "bad CRC with missing neighbour: not trusted");
	ring_lo = 13900;

	// drive shifted by one frame: Q stored at lba-1 belongs to lba ... learn offset
	drive_offset = 1;
	int wrong = 0;
	for (int fr = 14120; fr < 14160; fr++) { st = psx_subq_get(fr, q); if (st && q_abs_frame(q) != fr) wrong++; }
	CHECK(wrong == 0, "shifted drive: never passes a Q of the wrong frame");
	st = psx_subq_get(14170, q);
	CHECK(st == 3 && q_abs_frame(q) == 14170 && s_q_offset == 1, "shifted drive: offset learned, real Q delivered again");
	drive_offset = 0;
	for (int fr = 14180; fr < 14220; fr++) psx_subq_get(fr, q);
	CHECK(s_q_offset == 0, "offset re-learned back to 0");

	// SPI framing of CD_SET sent before a sector
	g_tx.clear();
	psx_subq_on_sector(14098); // Q of frame 14100
	CHECK(g_tx.size() == 1 && g_tx[0].size() == 9 && g_tx[0][0] == 0x35, "CD_SET: command + 8 words");
	CHECK(g_tx[0][1] == (14100 & 0xFFFF) && g_tx[0][2] == ((3 << 8) | (14100 >> 16)), "CD_SET: tag and status");
	CHECK((g_tx[0][3] & 0xFF) == 0x41 && (g_tx[0][3] >> 8) == 0x01, "CD_SET: Q bytes little endian");

	// CD_GET poll: new physical request and GetQ request
	g_tx.clear();
	g_cdget_resp[0] = (1 << 8) | 1; g_cdget_resp[1] = 14200 & 0xFFFF; g_cdget_resp[2] = 14200 >> 16; g_cdget_resp[3] = 0x01A2;
	psx_subq_poll();
	CHECK(g_tx.size() == 3, "poll: CD_GET + 2 answers");
	CHECK(g_tx.size() == 3 && g_tx[1][1] == (14200 & 0xFFFF) && (g_tx[1][2] >> 8) == 3, "poll: physical request answered with real Q");
	CHECK(g_tx.size() == 3 && g_tx[2][1] == 0x01A2 && (g_tx[2][2] >> 8) == 0x07 && (g_tx[2][6] >> 8) == 0x60, "poll: GetQ A2 answered from lead-in");
	g_tx.clear();
	psx_subq_poll();
	CHECK(g_tx.size() == 1, "poll: no new request -> only CD_GET");
	g_cdget_resp[0] = (2 << 8) | 1; g_cdget_resp[3] = 0x0155;
	g_tx.clear();
	psx_subq_poll();
	CHECK(g_tx.size() == 2 && (g_tx[1][2] >> 8) == 0x0C, "poll: unknown point -> LEADIN|NOTFOUND");

	psx_subq_reset();
	CHECK(!psx_subq_enabled() && psx_subq_get(14100, q) == 0, "reset: disabled");

	printf(fails ? "%d TESTS FAILED\n" : "ALL TESTS PASSED\n", fails);
	return fails != 0;
}

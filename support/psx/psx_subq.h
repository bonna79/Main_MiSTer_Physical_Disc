#ifndef PSX_SUBQ_H
#define PSX_SUBQ_H

#include <stdint.h>
#include "../../cd.h"
#include "../../file_io.h"

// Real Subchannel Q for the PSX core ("real-subq").
//
// The HPS delivers to the core the Subchannel Q really stored on the disc (physical disc
// through the USB drive, CHD with subcode, CloneCD .sub next to the .cue, or the
// modified Q frames of an .sbi file) instead of letting the core synthesize it from the TOC.
// It also answers the GetQ (1Dh) lead-in queries.
//
// Protocol (hps_ext.v of the PSX core):
//   CD_SET 0x35: tag(24 bit frame or adr/point) + status + 12 Q bytes, sent right before each
//                sector (Q of frame lba+2) and as answer to core requests
//   CD_GET 0x34: {getq_seq, phys_seq}, phys_tag, getq adr/point (polled from psx_poll)
// Cores without real-subq ignore both commands and the enable bit, so the Main stays
// compatible with the upstream PSX core.

#define PSX_SUBQ_ST_PRESENT   0x01
#define PSX_SUBQ_ST_CRCOK     0x02
#define PSX_SUBQ_ST_LEADIN    0x04
#define PSX_SUBQ_ST_NOTFOUND  0x08

// set up after a disc/image was loaded into toc. image may be NULL (physical disc),
// sbi may be NULL or an open .sbi file (read from the start).
void psx_subq_setup(toc_t *toc, int phys, const char *image, fileTYPE *sbi);
void psx_subq_reset(void);
int  psx_subq_enabled(void);

// Q for an absolute frame (MSF frame number, i.e. LBA + 150). Returns status bits, q[12] filled.
int  psx_subq_get(int frame, uint8_t *q);

// called by user_io right before a sector requested by the core is sent
void psx_subq_on_sector(uint32_t lba);

// called from psx_poll(): serves core requests (replaces the plain CD_GET heartbeat poll)
void psx_subq_poll(void);

#endif

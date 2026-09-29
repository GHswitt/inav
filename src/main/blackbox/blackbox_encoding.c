/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdlib.h>
#include <stdarg.h>
#include <string.h>

#include "platform.h"

#ifdef USE_BLACKBOX

#include "blackbox_encoding.h"
#include "blackbox_io.h"

#include "common/encoding.h"
#include "common/printf.h"

/* Longest header line this writer emits. "Firmware revision" is the longest as of
 * writing; 128 leaves generous room. Field-name lines are far longer but go out
 * through blackboxWriteFieldHeader(), not this function. */
#define BLACKBOX_HEADER_LINE_MAX 128


static void _putc(void *p, char c)
{
    (void)p;
    blackboxWrite(c);
}

static int blackboxPrintfv(const char *fmt, va_list va)
{
    return tfp_format(NULL, _putc, fmt, va);
}


//printf() to the blackbox serial port with no blocking shenanigans (so it's caller's responsibility to not write too fast!)
int blackboxPrintf(const char *fmt, ...)
{
    va_list va;

    va_start(va, fmt);

    const int written = blackboxPrintfv(fmt, va);

    va_end(va);

    return written;
}

/*
 * printf a Blackbox header line with a leading "H " and trailing "\n" added automatically. blackboxHeaderBudget is
 * decreased to account for the number of bytes written.
 */
/* A putf() sink that appends to a fixed buffer and silently discards anything
 * past the end, so rendering can never overrun lineBuf. */
typedef struct blackboxLineSink_s {
    char *buf;
    uint16_t len;
    uint16_t cap;
} blackboxLineSink_t;

static void blackboxLinePutc(void *p, char c)
{
    blackboxLineSink_t *sink = (blackboxLineSink_t *)p;
    if (sink->len < sink->cap) {
        sink->buf[sink->len++] = c;
    }
}

/*
 * Render the line into a buffer and write it with blackboxWriteChunk(), resuming
 * from wherever the previous attempt stopped.
 *
 * Previously each byte went out through blackboxWrite(), which cannot report
 * failure: on SD it calls afatfs_fputc(), which drops the byte silently whenever
 * the filesystem is busy -- and the filesystem is busiest exactly while the log
 * file is being created and extended, which is when headers are written. The
 * caller had already advanced xmitState by then, so dropped bytes were never
 * re-sent and the header lost runs of bytes mid-line.
 *
 * Returns true when the whole line has been written. On false the caller must
 * NOT advance its state machine; call again next iteration and the remainder of
 * the same line is sent. The file cursor only advances by what was accepted, so
 * resuming at the offset reproduces the line exactly.
 */
/* Shared by both resumable writers. Sharing one buffer is safe because no caller
 * advances its state machine while a line is incomplete, so two partial lines can
 * never be in flight at once. */
static char resumeBuf[BLACKBOX_HEADER_LINE_MAX];
static uint16_t resumeLen = 0;
static uint16_t resumeSent = 0;

/* Push whatever the device will take. Returns true once the buffer is drained,
 * reporting through *outLen how many bytes the completed line occupied. */
static bool blackboxResumableFlush(uint16_t *outLen)
{
    resumeSent += blackboxWriteChunk((const uint8_t *)resumeBuf + resumeSent, resumeLen - resumeSent);

    if (resumeSent < resumeLen) {
        return false;
    }

    *outLen = resumeLen;
    resumeLen = 0;
    return true;
}

bool blackboxPrintfHeaderLine(const char *name, const char *fmt, ...)
{
    if (resumeLen == 0) {
        // Note tfp_snprintf() cannot be used to render this: its size argument is
        // not honoured (printf.c computes its end pointer from the address of its
        // own local pointer, not from the caller's buffer), so it would overrun.
        // Render through tfp_format() with a sink that bounds itself. One byte is
        // held back for the trailing newline.
        blackboxLineSink_t sink = { .buf = resumeBuf, .len = 0, .cap = sizeof(resumeBuf) - 1 };

        blackboxLinePutc(&sink, 'H');
        blackboxLinePutc(&sink, ' ');
        for (const char *p = name; *p != '\0'; p++) {
            blackboxLinePutc(&sink, *p);
        }
        blackboxLinePutc(&sink, ':');

        va_list va;
        va_start(va, fmt);
        tfp_format(&sink, blackboxLinePutc, fmt, va);
        va_end(va);

        // Always fits: cap left one byte spare. An over-long line is truncated
        // rather than dropped, so the log stays parseable.
        resumeBuf[sink.len++] = '\n';

        resumeLen = sink.len;
        resumeSent = 0;
    }

    uint16_t written;
    if (!blackboxResumableFlush(&written)) {
        return false;               // retry the remainder next iteration
    }

    blackboxHeaderBudget -= written;
    return true;
}

/*
 * As blackboxPrintfHeaderLine(), but writes the formatted text verbatim -- no
 * "H name:" prefix and no newline -- for callers that build a header line from
 * several pieces. The caller owns blackboxHeaderBudget.
 *
 * Returns true when the whole piece has been written. On false the caller must
 * not advance its state machine; call again next iteration and the remainder of
 * the same piece is sent. Arguments are ignored while a piece is pending, so it
 * is safe (and required) to call again with the same ones.
 */
bool blackboxPrintfResumable(const char *fmt, ...)
{
    if (resumeLen == 0) {
        blackboxLineSink_t sink = { .buf = resumeBuf, .len = 0, .cap = sizeof(resumeBuf) };

        va_list va;
        va_start(va, fmt);
        tfp_format(&sink, blackboxLinePutc, fmt, va);
        va_end(va);

        resumeLen = sink.len;
        resumeSent = 0;
    }

    uint16_t written;
    return blackboxResumableFlush(&written);
}

/**
 * Write an unsigned integer to the blackbox serial port using variable byte encoding.
 */
void blackboxWriteUnsignedVB(uint32_t value)
{
    //While this isn't the final byte (we can only write 7 bits at a time)
    while (value > 127) {
        blackboxWrite((uint8_t) (value | 0x80)); // Set the high bit to mean "more bytes follow"
        value >>= 7;
    }
    blackboxWrite(value);
}

/**
 * Write a signed integer to the blackbox serial port using ZigZig and variable byte encoding.
 */
void blackboxWriteSignedVB(int32_t value)
{
    //ZigZag encode to make the value always positive
    blackboxWriteUnsignedVB(zigzagEncode(value));
}

void blackboxWriteSignedVBArray(int32_t *array, int count)
{
    for (int i = 0; i < count; i++) {
        blackboxWriteSignedVB(array[i]);
    }
}

void blackboxWriteSigned16VBArray(int16_t *array, int count)
{
    for (int i = 0; i < count; i++) {
        blackboxWriteSignedVB(array[i]);
    }
}

void blackboxWriteS16(int16_t value)
{
    blackboxWrite(value & 0xFF);
    blackboxWrite((value >> 8) & 0xFF);
}

/**
 * Write a 2 bit tag followed by 3 signed fields of 2, 4, 6 or 32 bits
 */
void blackboxWriteTag2_3S32(int32_t *values)
{
    static const int NUM_FIELDS = 3;

    //Need to be enums rather than const ints if we want to switch on them (due to being C)
    enum {
        BITS_2  = 0,
        BITS_4  = 1,
        BITS_6  = 2,
        BITS_32 = 3
    };

    enum {
        BYTES_1  = 0,
        BYTES_2  = 1,
        BYTES_3  = 2,
        BYTES_4  = 3
    };

    int selector = BITS_2, selector2;

    /*
     * Find out how many bits the largest value requires to encode, and use it to choose one of the packing schemes
     * below:
     *
     * Selector possibilities
     *
     * 2 bits per field  ss11 2233,
     * 4 bits per field  ss00 1111 2222 3333
     * 6 bits per field  ss11 1111 0022 2222 0033 3333
     * 32 bits per field sstt tttt followed by fields of various byte counts
     */
    for (int x = 0; x < NUM_FIELDS; x++) {
        //Require more than 6 bits?
        if (values[x] >= 32 || values[x] < -32) {
            selector = BITS_32;
            break;
        }

        //Require more than 4 bits?
        if (values[x] >= 8 || values[x] < -8) {
             if (selector < BITS_6) {
                 selector = BITS_6;
             }
        } else if (values[x] >= 2 || values[x] < -2) { //Require more than 2 bits?
            if (selector < BITS_4) {
                selector = BITS_4;
            }
        }
    }

    switch (selector) {
    case BITS_2:
        blackboxWrite((selector << 6) | ((values[0] & 0x03) << 4) | ((values[1] & 0x03) << 2) | (values[2] & 0x03));
        break;
    case BITS_4:
        blackboxWrite((selector << 6) | (values[0] & 0x0F));
        blackboxWrite((values[1] << 4) | (values[2] & 0x0F));
        break;
    case BITS_6:
        blackboxWrite((selector << 6) | (values[0] & 0x3F));
        blackboxWrite((uint8_t)values[1]);
        blackboxWrite((uint8_t)values[2]);
        break;
    case BITS_32:
        /*
         * Do another round to compute a selector for each field, assuming that they are at least 8 bits each
         *
         * Selector2 field possibilities
         * 0 - 8 bits
         * 1 - 16 bits
         * 2 - 24 bits
         * 3 - 32 bits
         */
        selector2 = 0;

        //Encode in reverse order so the first field is in the low bits:
        for (int x = NUM_FIELDS - 1; x >= 0; x--) {
            selector2 <<= 2;

            if (values[x] < 128 && values[x] >= -128) {
                selector2 |= BYTES_1;
            } else if (values[x] < 32768 && values[x] >= -32768) {
                selector2 |= BYTES_2;
            } else if (values[x] < 8388608 && values[x] >= -8388608) {
                selector2 |= BYTES_3;
            } else {
                selector2 |= BYTES_4;
            }
        }

        //Write the selectors
        blackboxWrite((selector << 6) | selector2);

        //And now the values according to the selectors we picked for them
        for (int x = 0; x < NUM_FIELDS; x++, selector2 >>= 2) {
            switch (selector2 & 0x03) {
            case BYTES_1:
                blackboxWrite(values[x]);
                break;
            case BYTES_2:
                blackboxWrite(values[x]);
                blackboxWrite(values[x] >> 8);
                break;
            case BYTES_3:
                blackboxWrite(values[x]);
                blackboxWrite(values[x] >> 8);
                blackboxWrite(values[x] >> 16);
                break;
            case BYTES_4:
                blackboxWrite(values[x]);
                blackboxWrite(values[x] >> 8);
                blackboxWrite(values[x] >> 16);
                blackboxWrite(values[x] >> 24);
                break;
            }
        }
        break;
    }
}

/**
 * Write an 8-bit selector followed by four signed fields of size 0, 4, 8 or 16 bits.
 */
void blackboxWriteTag8_4S16(int32_t *values)
{

    //Need to be enums rather than const ints if we want to switch on them (due to being C)
    enum {
        FIELD_ZERO  = 0,
        FIELD_4BIT  = 1,
        FIELD_8BIT  = 2,
        FIELD_16BIT = 3
    };

    uint8_t selector = 0;
    //Encode in reverse order so the first field is in the low bits:
    for (int x = 3; x >= 0; x--) {
        selector <<= 2;

        if (values[x] == 0) {
            selector |= FIELD_ZERO;
        } else if (values[x] < 8 && values[x] >= -8) {
            selector |= FIELD_4BIT;
        } else if (values[x] < 128 && values[x] >= -128) {
            selector |= FIELD_8BIT;
        } else {
            selector |= FIELD_16BIT;
        }
    }

    blackboxWrite(selector);

    int nibbleIndex = 0;
    uint8_t buffer = 0;
    for (int x = 0; x < 4; x++, selector >>= 2) {
        switch (selector & 0x03) {
        case FIELD_ZERO:
            //No-op
            break;
        case FIELD_4BIT:
            if (nibbleIndex == 0) {
                //We fill high-bits first
                buffer = values[x] << 4;
                nibbleIndex = 1;
            } else {
                blackboxWrite(buffer | (values[x] & 0x0F));
                nibbleIndex = 0;
            }
            break;
        case FIELD_8BIT:
            if (nibbleIndex == 0) {
                blackboxWrite(values[x]);
            } else {
                //Write the high bits of the value first (mask to avoid sign extension)
                blackboxWrite(buffer | ((values[x] >> 4) & 0x0F));
                //Now put the leftover low bits into the top of the next buffer entry
                buffer = values[x] << 4;
            }
            break;
        case FIELD_16BIT:
            if (nibbleIndex == 0) {
                //Write high byte first
                blackboxWrite(values[x] >> 8);
                blackboxWrite(values[x]);
            } else {
                //First write the highest 4 bits
                blackboxWrite(buffer | ((values[x] >> 12) & 0x0F));
                // Then the middle 8
                blackboxWrite(values[x] >> 4);
                //Only the smallest 4 bits are still left to write
                buffer = values[x] << 4;
            }
            break;
        }
    }
    //Anything left over to write?
    if (nibbleIndex == 1) {
        blackboxWrite(buffer);
    }
}

/**
 * Write `valueCount` fields from `values` to the Blackbox using signed variable byte encoding. A 1-byte header is
 * written first which specifies which fields are non-zero (so this encoding is compact when most fields are zero).
 *
 * valueCount must be 8 or less.
 */
void blackboxWriteTag8_8SVB(int32_t *values, int valueCount)
{
    uint8_t header;

    if (valueCount > 0) {
        //If we're only writing one field then we can skip the header
        if (valueCount == 1) {
            blackboxWriteSignedVB(values[0]);
        } else {
            //First write a one-byte header that marks which fields are non-zero
            header = 0;

            // First field should be in low bits of header
            for (int i = valueCount - 1; i >= 0; i--) {
                header <<= 1;

                if (values[i] != 0) {
                    header |= 0x01;
                }
            }

            blackboxWrite(header);

            for (int i = 0; i < valueCount; i++) {
                if (values[i] != 0) {
                    blackboxWriteSignedVB(values[i]);
                }
            }
        }
    }
}

/** Write unsigned integer **/
void blackboxWriteU32(int32_t value)
{
    blackboxWrite(value & 0xFF);
    blackboxWrite((value >> 8) & 0xFF);
    blackboxWrite((value >> 16) & 0xFF);
    blackboxWrite((value >> 24) & 0xFF);
}

/** Write float value in the integer form **/
void blackboxWriteFloat(float value)
{
    blackboxWriteU32(castFloatBytesToInt(value));
}
#endif // BLACKBOX

/*
 * Copyright (c) 2024 Kelly Helmut Lord
 * Copyright (c) 2025 Martin Schröder
 * Copyright (c) 2026 Basalte bv
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <sys/types.h>
#include <zephyr/sys/util.h>
#include <zephyr/data/cobs.h>

static int cobs_net_buf_cb(const uint8_t *buf, size_t len, void *user_data)
{
	struct net_buf *dst = user_data;

	if (net_buf_tailroom(dst) < len) {
		return -ENOMEM;
	}

	(void)net_buf_add_mem(dst, buf, len);

	return 0;
}

int cobs_encode(struct net_buf *src, struct net_buf *dst, uint32_t flags)
{
	struct cobs_encoder enc;
	size_t len = src->len;
	int ret;

	(void)cobs_encoder_init(&enc, cobs_net_buf_cb, dst, flags);

	ret = cobs_encoder_write(&enc, net_buf_pull_mem(src, len), len);
	if (ret < 0) {
		return ret;
	}

	return cobs_encoder_close(&enc);
}

int cobs_decode(struct net_buf *src, struct net_buf *dst, uint32_t flags)
{
	struct cobs_decoder dec;
	size_t len = src->len;
	int ret;

	(void)cobs_decoder_init(&dec, cobs_net_buf_cb, dst, flags);

	ret = cobs_decoder_write(&dec, net_buf_pull_mem(src, len), len);
	if (ret < 0) {
		return ret;
	}

	return cobs_decoder_close(&dec);
}

static inline void cobs_encoder_reset(struct cobs_encoder *enc)
{
	/* Reset buffer */
	enc->fragment[0] = 1;
}

static int cobs_encoder_finish(struct cobs_encoder *enc, bool close)
{
	uint8_t sentinel = COBS_FLAG_CUSTOM_DELIMITER(enc->flags);
	size_t len = enc->fragment[0];
	int ret;

	if (sentinel != 0x00) {
		for (size_t i = 0; i < len; ++i) {
			enc->fragment[i] ^= sentinel;
		}
	}

	ret = enc->cb(enc->fragment, len, enc->cb_user_data);
	if (ret < 0) {
		cobs_encoder_reset(enc);
		return ret;
	}

	if (close && (enc->flags & COBS_FLAG_TRAILING_DELIMITER) != 0U) {
		ret = enc->cb(&sentinel, 1, enc->cb_user_data);
		if (ret < 0) {
			cobs_encoder_reset(enc);
			return ret;
		}
	}

	cobs_encoder_reset(enc);

	return 0;
}

int cobs_encoder_init(struct cobs_encoder *enc, cobs_stream_cb cb, void *user_data, uint32_t flags)
{
	if (cb == NULL) {
		return -EINVAL;
	}

	__ASSERT_NO_MSG(enc != NULL);

	enc->cb = cb;
	enc->cb_user_data = user_data;
	enc->flags = flags;

	cobs_encoder_reset(enc);

	return 0;
}

int cobs_encoder_close(struct cobs_encoder *enc)
{
	__ASSERT_NO_MSG(enc != NULL);

	return cobs_encoder_finish(enc, true);
}

int cobs_encoder_write(struct cobs_encoder *enc, const uint8_t *buf, size_t len)
{
	int ret;

	__ASSERT_NO_MSG(enc != NULL);
	__ASSERT_NO_MSG(len <= INT_MAX);

	for (size_t i = 0; i < len; ++i) {
		/* Finish if group is full */
		if (enc->fragment[0] == 0xff) {
			ret = cobs_encoder_finish(enc, false);
			if (ret < 0) {
				return ret;
			}
		}

		if (buf[i] == 0x00) {
			ret = cobs_encoder_finish(enc, false);
			if (ret < 0) {
				return ret;
			}

			continue;
		}

		enc->fragment[enc->fragment[0]] = buf[i];
		enc->fragment[0]++;
	}

	return len;
}

static inline void cobs_decoder_reset(struct cobs_decoder *dec)
{
	dec->code = 0xff;
	dec->code_index = 0;
}

static inline bool cobs_decoder_needs_more_data(struct cobs_decoder *dec)
{
	return dec->code_index != 0;
}

int cobs_decoder_init(struct cobs_decoder *dec, cobs_stream_cb cb, void *user_data, uint32_t flags)
{
	if (cb == NULL) {
		return -EINVAL;
	}

	__ASSERT_NO_MSG(dec != NULL);

	dec->cb = cb;
	dec->cb_user_data = user_data;
	dec->flags = flags;

	cobs_decoder_reset(dec);

	return 0;
}

int cobs_decoder_close(struct cobs_decoder *dec)
{
	int ret;

	__ASSERT_NO_MSG(dec != NULL);

	ret = cobs_decoder_needs_more_data(dec) ? -EINVAL : 0;
	cobs_decoder_reset(dec);

	return ret;
}

int cobs_decoder_write(struct cobs_decoder *dec, const uint8_t *buf, size_t len)
{
	uint8_t sentinel = COBS_FLAG_CUSTOM_DELIMITER(dec->flags);
	int ret;

	__ASSERT_NO_MSG(dec != NULL);
	__ASSERT_NO_MSG(len <= INT_MAX);

	for (size_t i = 0; i < len; ++i) {
		uint8_t data = buf[i] ^ sentinel;

		if (data == 0x00) {
			if ((dec->flags & COBS_FLAG_TRAILING_DELIMITER) == 0U ||
			    cobs_decoder_needs_more_data(dec)) {
				/* Decoder shouldn't get delimiters or unexpected end of data */
				cobs_decoder_reset(dec);
				return -EINVAL;
			}

			/* Notify frame delimiter was seen */
			ret = dec->cb(NULL, 0, dec->cb_user_data);
			if (ret < 0) {
				cobs_decoder_reset(dec);
				return ret;
			}

			/* Reset state */
			cobs_decoder_reset(dec);
			continue;
		}

		if (dec->code_index > 0) {
			ret = dec->cb(&data, 1, dec->cb_user_data);
			if (ret < 0) {
				cobs_decoder_reset(dec);
				return ret;
			}

			dec->code_index--;
			continue;
		}

		dec->code_index = data;

		if (dec->code != 0xff) {
			/* Group finished, output zero byte */
			data = 0x00;

			ret = dec->cb(&data, 1, dec->cb_user_data);
			if (ret < 0) {
				cobs_decoder_reset(dec);
				return ret;
			}
		}

		dec->code = dec->code_index;
		dec->code_index--;
	}

	return len;
}

/* Streaming Encoder */

void cobs_encode_init(struct cobs_encode_state *self)
{
	__ASSERT(self != NULL, "self must not be NULL");

	self->src_frag = NULL;
	self->src_offset = 0;
	self->block_code = 0;
	self->block_pos = 0;
}

/* Skip empty fragments and peek at current byte */
static inline bool peek_byte(struct cobs_encode_state *self, uint8_t *byte_out)
{
	while ((self->src_frag != NULL) && (self->src_offset >= self->src_frag->len)) {
		self->src_frag = self->src_frag->frags;
		self->src_offset = 0;
	}

	if (self->src_frag == NULL || self->src_offset >= self->src_frag->len) {
		return false;
	}

	*byte_out = self->src_frag->data[self->src_offset];
	return true;
}

static inline void advance_byte(struct cobs_encode_state *self)
{
	self->src_offset++;
}

/* Scan ahead to find code byte value */
static uint8_t scan_for_code_byte(const struct cobs_encode_state *self,
				  const uint8_t delimiter)
{
	struct net_buf *frag = self->src_frag;
	size_t offset = self->src_offset;
	uint8_t count = 1;

	while ((frag != NULL) && (count < 0xFF)) {
		while (offset < frag->len && count < 0xFF) {
			if (frag->data[offset] == delimiter) {
				return count;
			}
			offset++;
			count++;
		}

		if (offset >= frag->len) {
			frag = frag->frags;
			offset = 0;
		}
	}

	return count;
}

int cobs_encode_stream(struct cobs_encode_state *self, struct net_buf *src,
		       uint8_t *dst, size_t *dst_len, uint8_t delimiter)
{
	if (self == NULL || src == NULL || dst == NULL || dst_len == NULL) {
		return -EINVAL;
	}

	const size_t capacity = *dst_len;
	size_t written = 0;
	bool last_block_ended_with_delimiter = false;

	/* Initialize source on first call */
	if (self->src_frag == NULL) {
		self->src_frag = src;
		self->src_offset = 0;
	}

	uint8_t byte;

	while ((peek_byte(self, &byte) != false) && (written < capacity)) {
		/* Start new block if needed */
		if (self->block_pos == 0) {
			if (written >= capacity) {
				break;
			}
			self->block_code = scan_for_code_byte(self, delimiter);
			dst[written++] = self->block_code;
		}

		/* Copy data bytes for this block */
		while ((peek_byte(self, &byte) != false) &&
		       (self->block_pos < self->block_code - 1) &&
		       (written < capacity)) {

			if (byte == delimiter) {
				advance_byte(self);
				self->block_pos = 0;
				break;
			}

			dst[written++] = byte;
			advance_byte(self);
			self->block_pos++;
		}

		/* Check if block is complete */
		if (self->block_pos == self->block_code - 1) {
			last_block_ended_with_delimiter = false;

			if ((self->block_code != 0xFF) && (peek_byte(self, &byte) != false) &&
			    (byte == delimiter)) {
				advance_byte(self);
				last_block_ended_with_delimiter = true;
			}
			self->block_pos = 0;
		}
	}

	/* Store whether we need final code byte in finalize */
	self->src_frag = (last_block_ended_with_delimiter != false) ? src : NULL;

	*dst_len = written;
	return 0;
}

int cobs_encode_finalize(struct cobs_encode_state *self, uint8_t *dst, size_t *dst_len,
			 uint8_t delimiter)
{
	if (self == NULL || dst == NULL || dst_len == NULL) {
		return -EINVAL;
	}

	size_t written = 0;
	size_t capacity = *dst_len;

	/* Write final code byte if last block ended with delimiter */
	if (self->src_frag != NULL) {
		if (capacity < 1) {
			return -ENOMEM;
		}
		dst[written++] = 0x01;
	}

	/* Reset state */
	self->block_code = 0;
	self->block_pos = 0;
	self->src_frag = NULL;
	self->src_offset = 0;

	*dst_len = written;
	return 0;
}

/* Streaming Decoder */

void cobs_decode_init(struct cobs_decode_state *self)
{
	__ASSERT(self != NULL, "self must not be NULL");

	self->bytes_left = 0;
	self->need_delimiter = false;
	self->frame_complete = false;
}

static inline int check_dst_space(const struct net_buf *dst)
{
	return (net_buf_tailroom(dst) < 1) ? -ENOMEM : 0;
}

/* Process code byte and insert delimiter if needed */
static inline int process_code_byte(struct cobs_decode_state *self,
				    const uint8_t *src, size_t *processed,
				    struct net_buf *dst,
				    const uint8_t delimiter)
{
	/* Insert pending delimiter before reading new code */
	if (self->need_delimiter != false) {
		if (src[*processed] == delimiter) {
			(*processed)++;
			self->need_delimiter = false;
			self->frame_complete = true;
			return 1;
		}

		int ret = check_dst_space(dst);

		if (ret != 0) {
			return ret;
		}

		net_buf_add_u8(dst, delimiter);
		self->need_delimiter = false;
	}

	/* Read code byte */
	uint8_t code = src[(*processed)++];

	if (code == delimiter) {
		self->frame_complete = true;
		return 1;
	}

	if (code == 0) {
		return -EINVAL;
	}

	self->bytes_left = code - 1;
	self->need_delimiter = (code != 0xFF);

	return 0;
}

int cobs_decode_stream(struct cobs_decode_state *self, const uint8_t *src,
		       size_t src_len, struct net_buf *dst, uint8_t delimiter)
{
	if (self == NULL || src == NULL || dst == NULL) {
		return -EINVAL;
	}

	size_t processed = 0;
	self->frame_complete = false;

	while (processed < src_len) {
		/* Read new code byte if needed */
		if (self->bytes_left == 0) {
			int ret = process_code_byte(self, src, &processed, dst, delimiter);
			if (ret == 1) {
				return (ssize_t)processed;
			}
			if (ret < 0) {
				return ret;
			}
		}

		/* Copy data bytes from block */
		while ((self->bytes_left > 0) && (processed < src_len)) {
			uint8_t byte = src[processed++];

			if (byte == delimiter) {
				return -EINVAL;
			}

			int ret = check_dst_space(dst);
			if (ret != 0) {
				return ret;
			}

			net_buf_add_u8(dst, byte);
			self->bytes_left--;
		}
	}

	return (ssize_t)processed;
}

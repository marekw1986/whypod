/*
 * audio.c
 *
 *  Created on: 8 kwi 2021
 *      Author: marek
 */

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include "ff.h"
#include "mad.h"
#include "cmsis_os.h"

#define PCM_BUF_SIZE 1152

typedef struct {
	uint32_t left[PCM_BUF_SIZE];
	uint32_t right[PCM_BUF_SIZE];
	uint16_t len;
	uint16_t rate;
} pcmbuffer_t;

pcmbuffer_t buf[2];
volatile uint8_t nr_buf;		// indeks aktywnego buforu

struct buffer {
	unsigned char const *start;
	unsigned long length;
};

FIL file;

struct buffer buffer;
uint8_t MP3Buffer[2048];


int decode(unsigned char const *start, unsigned long length);


void mp3_play ( const char *fn ) {
	FRESULT res;

	res = f_open(&file, fn, FA_READ);
	if (res != FR_OK) {
		printf("File open error\r\n");
		return;
	}
}


void mp3_handle (void) {
	//int result;

	decode(MP3Buffer, sizeof(MP3Buffer));
	//result = mad_decoder_run(&decoder, MAD_DECODER_MODE_SYNC);
}


enum mad_flow mad_input_callback(void *data, struct mad_stream *stream)	{
	//struct buffer *buffer = data;
	FRESULT res;
	uint8_t	*destination;
	uint32_t rest;
	uint32_t bytes_to_load;
	UINT br;

	destination = MP3Buffer;
	rest = stream->bufend - stream->next_frame;

	printf("MAD input\r\n");
	if (stream->buffer) {
		//Second run
		memcpy((void*)MP3Buffer, stream->next_frame, rest);
		destination += rest;
	}

	bytes_to_load = sizeof(MP3Buffer) - rest;
	res = f_read(&file, destination, bytes_to_load, &br);
	if (res != FR_OK) {
		printf("Cand read MP3 data from file. Error: %d", res);
		return MAD_FLOW_BREAK;
	}

	if (br < bytes_to_load) {
		f_lseek(&file, 0);
	}

	//if (!buffer->length) return MAD_FLOW_STOP;
	mad_stream_buffer(stream, MP3Buffer, rest+br);
	//buffer->length = 0;

	return MAD_FLOW_CONTINUE;
}


enum mad_flow mad_output_callback(void *data, struct mad_header const *header, struct mad_pcm *pcm) {
	//struct buffer *buffer = data;
	mad_fixed_t const *samples;
	uint16_t nsamples, rate;

    printf("MP3 frame decoded\r\n");
    //printf("Sample rate: %d\r\n", pcm->samplerate);
    //printf("Channels: %d\r\n", pcm->channels);
    //printf("Length: %d\r\n", pcm->length);
    osDelay(1000);

    //xSemaphoreTake(xPCMSemaphore, portMAX_DELAY);
	nsamples = pcm->length;
	rate = pcm->samplerate;
	samples = pcm->samples[0];
	memcpy(buf[nr_buf ^ 0x01].left, samples, nsamples);
	samples = pcm->samples[1];
	memcpy(buf[nr_buf ^ 0x01].right, samples, nsamples);

	buf[nr_buf ^ 0x01].len = nsamples;
	buf[nr_buf ^ 0x01].rate = rate;

	return MAD_FLOW_CONTINUE;
}


enum mad_flow mad_error_callback(void *data, struct mad_stream *stream, struct mad_frame *frame) {
    struct buffer *buffer = data;

    printf("decoding error 0x%04x (%s) at byte offset %u\n", stream->error, mad_stream_errorstr(stream), stream->this_frame - buffer->start);

    /* return MAD_FLOW_BREAK here to stop decoding (and propagate an error) */
    return MAD_FLOW_CONTINUE;
}


int decode(unsigned char const *start, unsigned long length) {
	int result;
	struct mad_decoder decoder;

	// initialize our private message structure
	buffer.start  = start;
	buffer.length = length;

	// configure input, output, and error functions
	mad_decoder_init(&decoder, &buffer, mad_input_callback, 0 /* header */, 0 /* filter */, mad_output_callback, mad_error_callback /* error */, 0 /* message */);

	// start decoding
	result = mad_decoder_run(&decoder, MAD_DECODER_MODE_SYNC);

	// release the decoder
	mad_decoder_finish(&decoder);

	return result;
}

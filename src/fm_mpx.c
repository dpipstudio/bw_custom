/*
    PiFmRds - FM/RDS transmitter for the Raspberry Pi
    Copyright (C) 2014 Christophe Jacquet, F8FTK
    Copyright (C) 2025 douxx@douxx.tech

    See: https://github.com/dpipstudio/BWCustom

    Original project: https://github.com/ChristopheJacquet/PiFmRds

    rds_wav.c is a test program that writes a RDS baseband signal to a WAV
    file. It requires libsndfile.

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.

    fm_mpx.c: generates an FM multiplex signal containing RDS plus possibly
    monaural or stereo audio.
*/

#include <sndfile.h>
#include <stdlib.h>
#include <strings.h>
#include <math.h>

#include "rds.h"

#define PI 3.141592654

#define FIR_HALF_SIZE 30
#define FIR_SIZE (2 * FIR_HALF_SIZE - 1)

static int loop_mode = 0;

size_t length;

// coefficients of the low-pass FIR filter
float low_pass_fir[FIR_HALF_SIZE];

float carrier_38[] = {0.0, 0.8660254037844386, 0.8660254037844388, 1.2246467991473532e-16, -0.8660254037844384, -0.8660254037844386};

float carrier_19[] = {0.0, 0.5, 0.8660254037844386, 1.0, 0.8660254037844388, 0.5, 1.2246467991473532e-16, -0.5, -0.8660254037844384, -1.0, -0.8660254037844386, -0.5};

int phase_38 = 0;
int phase_19 = 0;

float downsample_factor;
static int raw_mode = 0;           // indicate raw PCM mode
static int raw_samplerate = 44100; // default sample rate for raw input (standard)

float *audio_buffer;
int audio_index = 0;
int audio_len = 0;
float audio_pos;

float fir_buffer_mono[FIR_SIZE] = {0};
float fir_buffer_stereo[FIR_SIZE] = {0};
int fir_index = 0;
int channels;

SNDFILE *inf;

float *alloc_empty_buffer(size_t length)
{
    float *p = malloc(length * sizeof(float));
    if (p == NULL)
        return NULL;

    bzero(p, length * sizeof(float));

    return p;
}

int fm_mpx_open(char *filename, size_t len, int loop, int is_raw, int samplerate, int num_channels)
{
    length = len;
    loop_mode = loop;
    raw_mode = is_raw;
    
    int in_samplerate = 44100;

    if (filename != NULL)
    {
        if (is_raw)
        {
            if (filename[0] == '-')
            {
                inf = NULL;
                raw_samplerate = samplerate;
                in_samplerate = samplerate;
                printf("Using stdin for raw 16-bit PCM input at %d Hz.\n", raw_samplerate);
            }
            else
            {
                fprintf(stderr, "Error: Raw mode only supports stdin input.\n");
                return -1;
            }

            downsample_factor = 228000. / raw_samplerate;
            printf("Input: %d Hz, upsampling factor: %.2f\n", raw_samplerate, downsample_factor);

            // Use the specified number of channels instead of assuming 2
            channels = num_channels;
            if(channels > 1) {
                printf("Raw PCM: %d channels, 16-bit signed, generating stereo multiplex.\n", channels);
            } else {
                printf("Raw PCM: 1 channel, 16-bit signed, monophonic operation.\n");
            }
        }
        else
        {
            // WAV file mode - use libsndfile
            SF_INFO sfinfo;

            if (filename[0] == '-')
            {
                if (!(inf = sf_open_fd(fileno(stdin), SFM_READ, &sfinfo, 0)))
                {
                    fprintf(stderr, "Error: could not open stdin for audio input.\n");
                    return -1;
                }
                else
                {
                    printf("Using stdin for audio input.\n");
                }
            }
            else
            {
                if (!(inf = sf_open(filename, SFM_READ, &sfinfo)))
                {
                    fprintf(stderr, "Error: could not open input file %s.\n", filename);
                    return -1;
                }
                else
                {
                    printf("Using audio file: %s\n", filename);
                }
            }

            in_samplerate = sfinfo.samplerate;  // Get actual sample rate from WAV
            downsample_factor = 228000. / in_samplerate;

            printf("Input: %d Hz, upsampling factor: %.2f\n", in_samplerate, downsample_factor);

            channels = sfinfo.channels;
            if (channels > 1)
            {
                printf("%d channels, generating stereo multiplex.\n", channels);
            }
            else
            {
                printf("1 channel, monophonic operation.\n");
            }
        }

        // Create the low-pass FIR filter (uses in_samplerate which is now always defined)
        float cutoff_freq = 15000 * .8;
        if (in_samplerate / 2 < cutoff_freq)
            cutoff_freq = in_samplerate / 2 * .8;

        low_pass_fir[FIR_HALF_SIZE - 1] = 2 * cutoff_freq / 228000 / 2;
        // Here we divide this coefficient by two because it will be counted twice
        // when applying the filter

        // Only store half of the filter since it is symmetric
        for (int i = 1; i < FIR_HALF_SIZE; i++)
        {
            low_pass_fir[FIR_HALF_SIZE - 1 - i] =
                sin(2 * PI * cutoff_freq * i / 228000) / (PI * i) // sinc
                * (.54 - .46 * cos(2 * PI * (i + FIR_HALF_SIZE) / (2 * FIR_HALF_SIZE)));
            // Hamming window
        }
        printf("Created low-pass FIR filter for audio channels, with cutoff at %.1f Hz\n", cutoff_freq);

        audio_pos = downsample_factor;
        audio_buffer = alloc_empty_buffer(length * channels);
        if (audio_buffer == NULL)
            return -1;

    } // end if(filename != NULL)
    else
    {
        inf = NULL;
        // inf == NULL indicates that there is no audio
    }

    return 0;
}

// Read raw 16-bit PCM samples from stdin
int read_raw_pcm(float *buffer, int samples_needed)
{
    int16_t pcm_buffer[samples_needed * channels];
    size_t bytes_to_read = samples_needed * channels * sizeof(int16_t);
    size_t bytes_read = fread(pcm_buffer, 1, bytes_to_read, stdin);

    if (bytes_read == 0)
    {
        if (loop_mode)
        {
            // Can't loop stdin in raw mode
            fprintf(stderr, "End of stdin in raw mode (looping not supported)\n");
            return -1;
        }
        else
        {
            fprintf(stderr, "Audio stream ended, terminating transmission\n");
            return -1;
        }
    }

    int samples_read = bytes_read / (channels * sizeof(int16_t));

    // Convert 16-bit PCM to float (-1.0 to 1.0)
    for (int i = 0; i < samples_read * channels; i++)
    {
        buffer[i] = pcm_buffer[i] / 32768.0f;
    }

    return samples_read;
}

// samples provided by this function are in 0..10: they need to be divided by
// 10 after.
int fm_mpx_get_samples(float *mpx_buffer)
{
    // 1. Handle RDS data first
    get_rds_samples(mpx_buffer, length);

    // 2. Safety check: stop if no audio source is available
    if (inf == NULL && !raw_mode)
        return 0;

    // 3. Main processing loop
    for (int i = 0; i < length; i++)
    {
        // Only advance audio data when the upsampling counter (audio_pos) 
        // reaches the required threshold (downsample_factor)
        if (audio_pos >= downsample_factor)
        {
            audio_pos -= downsample_factor;

            // Move pointers forward for the next sample
            audio_index += channels;
            audio_len -= channels;

            // 4. BUFFER REFILL LOGIC
            // If we've hit the end of our current buffer, fetch more from the source
            if (audio_len <= 0)
            {
                if (raw_mode)
                {
                    // Read fresh 16-bit PCM from stdin
                    int samples_read = read_raw_pcm(audio_buffer, length);
                    if (samples_read <= 0) return -1;
                    
                    // Update metadata: read_raw_pcm returns sample count, 
                    // so we multiply by channels to get the float count.
                    audio_len = samples_read * channels;
                    audio_index = 0;
                }
                else
                {
                    // Standard WAV file reading via libsndfile
                    int read_count = sf_read_float(inf, audio_buffer, length * channels);
                    if (read_count <= 0)
                    {
                        if (loop_mode && sf_seek(inf, 0, SEEK_SET) >= 0)
                        {
                            read_count = sf_read_float(inf, audio_buffer, length * channels);
                            if (read_count <= 0) return -1;
                        }
                        else
                        {
                            return -1; // End of file
                        }
                    }
                    audio_len = read_count;
                    audio_index = 0;
                }
            }
        }

        // 5. STEREO/MONO MIXING
        // Pull data from the buffer and place it into the FIR ring buffer
        if (channels > 1)
        {
            // Stereo: Generate Sum (L+R) and Difference (L-R)
            fir_buffer_mono[fir_index] = audio_buffer[audio_index] + audio_buffer[audio_index + 1];
            fir_buffer_stereo[fir_index] = audio_buffer[audio_index] - audio_buffer[audio_index + 1];
        }
        else
        {
            // Mono: Use the single channel directly
            fir_buffer_mono[fir_index] = audio_buffer[audio_index];
        }

        // Advance ring buffer index
        fir_index++;
        if (fir_index >= FIR_SIZE)
            fir_index = 0;

        // 6. APPLY LOW-PASS FIR FILTER
        float out_mono = 0;
        float out_stereo = 0;
        int ifbi = fir_index; // Increasing index
        int dfbi = fir_index; // Decreasing index

        for (int fi = 0; fi < FIR_HALF_SIZE; fi++)
        {
            dfbi--;
            if (dfbi < 0) dfbi = FIR_SIZE - 1;

            out_mono += low_pass_fir[fi] * (fir_buffer_mono[ifbi] + fir_buffer_mono[dfbi]);
            
            if (channels > 1)
            {
                out_stereo += low_pass_fir[fi] * (fir_buffer_stereo[ifbi] + fir_buffer_stereo[dfbi]);
            }

            ifbi++;
            if (ifbi >= FIR_SIZE) ifbi = 0;
        }

        // 7. FM MULTIPLEX MODULATION
        // Add the Mono (L+R) signal to the RDS baseband
        mpx_buffer[i] += 4.05 * out_mono;

        if (channels > 1)
        {
            // Add Stereo Difference (L-R) on 38kHz subcarrier and 19kHz pilot tone
            mpx_buffer[i] += 4.05 * carrier_38[phase_38] * out_stereo + 0.9 * carrier_19[phase_19];

            phase_19 = (phase_19 + 1) % 12;
            phase_38 = (phase_38 + 1) % 6;
        }

        // Increment the upsampling master counter
        audio_pos++;
    }

    return 0;
}

int fm_mpx_close()
{
    if (inf != NULL)
    {
        if (sf_close(inf))
        {
            fprintf(stderr, "Error closing audio file");
        }
    }

    if (audio_buffer != NULL)
        free(audio_buffer);

    return 0;
}
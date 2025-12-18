#include "AudioPlayer.hpp"
#include <alsa/asoundlib.h>
#include <cstdio>
#include <iostream>

AudioPlayer::AudioPlayer() {}

AudioPlayer::~AudioPlayer() {}

void AudioPlayer::setFile(const std::string& filePath) {
    m_filePath = filePath;
}

// 播放音频
bool AudioPlayer::play() {
    if (m_filePath.empty()) {
        std::cerr << "Error: Audio file path is empty." << std::endl;
        return false;
    }

    int channels = 0, sampleRate = 0, bitsPerSample = 0;
    size_t dataSize = 0;
    FILE* file = nullptr;

    if (!loadWavHeader(channels, sampleRate, bitsPerSample, dataSize, file)) {
        return false;
    }

    bool result = playPCM(file, channels, sampleRate, bitsPerSample, dataSize);

    fclose(file);
    return result;
}

// 解析 WAV 文件头
bool AudioPlayer::loadWavHeader(int& channels, int& sampleRate, int& bitsPerSample, size_t& dataSize, FILE*& file) {
    file = fopen(m_filePath.c_str(), "rb");
    if (!file) {
        std::cerr << "Error: Cannot open file " << m_filePath << std::endl;
        return false;
    }

    char riff[4];
    fread(riff, 1, 4, file);
    if (std::string(riff, 4) != "RIFF") {
        std::cerr << "Error: Not a valid WAV file" << std::endl;
        fclose(file);
        return false;
    }

    fseek(file, 22, SEEK_SET);
    fread(&channels, sizeof(short), 1, file);

    fread(&sampleRate, sizeof(int), 1, file);

    fseek(file, 34, SEEK_SET);
    fread(&bitsPerSample, sizeof(short), 1, file);

    // 找到 data 块
    fseek(file, 36, SEEK_SET);
    char dataHeader[4];
    fread(dataHeader, 1, 4, file);
    while (std::string(dataHeader, 4) != "data") {
        int chunkSize = 0;
        fread(&chunkSize, sizeof(int), 1, file);
        fseek(file, chunkSize, SEEK_CUR);
        fread(dataHeader, 1, 4, file);
    }

    fread(&dataSize, sizeof(int), 1, file);

    return true;
}

// 播放 PCM 数据
bool AudioPlayer::playPCM(FILE* file, int channels, int sampleRate, int bitsPerSample, size_t dataSize) {
    snd_pcm_t* handle;
    snd_pcm_hw_params_t* params;

    int rc = snd_pcm_open(&handle, "default", SND_PCM_STREAM_PLAYBACK, 0);
    if (rc < 0) {
        std::cerr << "Unable to open PCM device: " << snd_strerror(rc) << std::endl;
        return false;
    }

    snd_pcm_hw_params_malloc(&params);
    snd_pcm_hw_params_any(handle, params);
    snd_pcm_hw_params_set_access(handle, params, SND_PCM_ACCESS_RW_INTERLEAVED);

    snd_pcm_format_t format;
    if (bitsPerSample == 16)
        format = SND_PCM_FORMAT_S16_LE;
    else if (bitsPerSample == 8)
        format = SND_PCM_FORMAT_U8;
    else {
        std::cerr << "Unsupported bits per sample: " << bitsPerSample << std::endl;
        snd_pcm_close(handle);
        return false;
    }

    snd_pcm_hw_params_set_format(handle, params, format);
    snd_pcm_hw_params_set_channels(handle, params, channels);
    snd_pcm_hw_params_set_rate(handle, params, sampleRate, 0);

    rc = snd_pcm_hw_params(handle, params);
    if (rc < 0) {
        std::cerr << "Unable to set HW parameters: " << snd_strerror(rc) << std::endl;
        snd_pcm_close(handle);
        return false;
    }

    const size_t bufferFrames = 1024;
    size_t frameSize = channels * bitsPerSample / 8;
    char* buffer = new char[bufferFrames * frameSize];

    size_t totalRead = 0;
    while (totalRead < dataSize) {
        size_t toRead = bufferFrames * frameSize;
        if (totalRead + toRead > dataSize)
            toRead = dataSize - totalRead;

        size_t read = fread(buffer, 1, toRead, file);
        if (read == 0)
            break;

        snd_pcm_writei(handle, buffer, read / frameSize);
        totalRead += read;
    }

    snd_pcm_drain(handle);
    snd_pcm_close(handle);
    delete[] buffer;

    return true;
}

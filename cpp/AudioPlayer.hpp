#ifndef AUDIOPLAYER_H
#define AUDIOPLAYER_H

#include <string>

class AudioPlayer {
public:
    AudioPlayer();
    ~AudioPlayer();

    // 设置音频文件路径
    void setFile(const std::string& filePath);

    // 播放音频（阻塞播放）
    bool play();

private:
    std::string m_filePath;

    // 解析 WAV 文件头
    bool loadWavHeader(int& channels, int& sampleRate, int& bitsPerSample, size_t& dataSize, FILE*& file);

    // 播放 PCM 数据
    bool playPCM(FILE* file, int channels, int sampleRate, int bitsPerSample, size_t dataSize);
};

#endif // AUDIOPLAYER_H

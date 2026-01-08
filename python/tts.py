import asyncio
import edge_tts
import os

class EdgeTTSPlayer:
    """
    Edge TTS 封装类，支持选择语音，输出 WAV
    """
    def __init__(self, voice="en-US-GuyNeural"):
        """
        初始化 TTS
        :param voice: 微软语音，可选 en-US-GuyNeural, en-US-AriaNeural, zh-CN-XiaoxiaoNeural 等
        """
        self.voice = voice

    async def synthesize(self, text, output_file="output.wav"):
        """
        生成 WAV 文件
        :param text: 文本内容
        :param output_file: 输出 WAV 文件路径
        """
        communicate = edge_tts.Communicate(text, self.voice)
        await communicate.save(output_file)
        return output_file

    def save(self, text, output_file="output.wav"):
        """
        同步接口封装（内部用 asyncio）
        """
        asyncio.run(self.synthesize(text, output_file))
        return output_file

# -----------------------------
# 使用示例
# -----------------------------
if __name__ == "__main__":
    text = "检测到链条缺陷啦,快来检查一下吧!"
    voice = "zh-CN-XiaoxiaoNeural"  
    output_file = "alert2.wav"

    tts_player = EdgeTTSPlayer(voice=voice)
    tts_player.save(text=text, output_file=output_file)
    print(f"WAV 文件已生成: {output_file}")

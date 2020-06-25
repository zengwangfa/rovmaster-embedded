
/**
 * @desc: MIC 音频驱动
 */
#include <alsa/asoundlib.h>
#include <elog.h>
#include <stdio.h>

#define DEFAULT_RECORD_DEVICE "default"
#define AUDIO_CHANNEL_SET 1  // 1单声道   2立体声
#define AUDIO_RATE_SET 44100 // 音频采样率,常用的采样频率: 44100Hz 、16000HZ、8000HZ、48000HZ、22050HZ

FILE *pcm_data_file = NULL;

int record_init(void)
{
    int err;
    char *buffer;
    int buffer_frames = 1024;
    unsigned int rate = AUDIO_RATE_SET;
    snd_pcm_t *handle;           // 一个指向PCM设备的句柄
    snd_pcm_hw_params_t *params; // 此结构包含有关硬件的信息，可用于指定PCM流的配置

    char *device = (char *)DEFAULT_RECORD_DEVICE;
    // SND_PCM_STREAM_PLAYBACK 输出流
    // SND_PCM_STREAM_CAPTURE  输入流
    err = snd_pcm_open(&handle, device, SND_PCM_STREAM_CAPTURE, 0);
    if (err)
        log_e("Unable to open PCM device: %s\n", snd_strerror(err));

    /*创建一个保存PCM数据的文件*/
    pcm_data_file = fopen("123.pcm", "wb");

    /*分配硬件参数结构对象*/
    snd_pcm_hw_params_malloc(&params);

    /*按照默认设置对硬件对象进行设置*/
    snd_pcm_hw_params_any(handle, params);

    /*设置数据为交叉模式*/
    snd_pcm_hw_params_set_access(handle, params, SND_PCM_ACCESS_RW_INTERLEAVED);

    /*设置数据编码格式*/
    snd_pcm_hw_params_set_format(handle, params, SND_PCM_FORMAT_S16_LE);

    /*设置采样频率*/
    snd_pcm_hw_params_set_rate_near(handle, params, &rate, 0);

    /*设置声道*/
    snd_pcm_hw_params_set_channels(handle, params, AUDIO_CHANNEL_SET);

    /*将配置写入驱动程序中*/
    snd_pcm_hw_params(handle, params);

    /*使采集卡处于空闲状态*/
    snd_pcm_hw_params_free(params);

    /*准备音频接口*/
    snd_pcm_prepare(handle);

    /*配置一个数据缓冲区用来缓冲数据*/
    int frame_byte = snd_pcm_format_width(SND_PCM_FORMAT_S16_LE) / 8;
    buffer = malloc(buffer_frames * frame_byte * AUDIO_CHANNEL_SET);

    while (1)
    {
        /*从声卡设备读取一帧音频数据:2048字节*/
        if ((err = snd_pcm_readi(handle, buffer, buffer_frames)) != buffer_frames)
        {
            printf("从音频接口读取失败(%s)\n", snd_strerror(err));
            exit(1);
        }
        /*写数据到文件: 音频的每帧数据样本大小是16位=2个字节*/
        fwrite(buffer, (buffer_frames * AUDIO_CHANNEL_SET), frame_byte, pcm_data_file);
    }

    /*释放数据缓冲区*/
    free(buffer);

    /*关闭音频采集卡硬件*/
    snd_pcm_close(handle);

    /*关闭文件流*/
    fclose(pcm_data_file);
    return 0;
}
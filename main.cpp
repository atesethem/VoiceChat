#include <iostream>
#include <opus/opus.h>
#include <portaudio.h>
#include <cmath>
#include <asio.hpp>
#include <vector>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <queue>

#define SAMPLE_RATE (48000) // 48khz 44.1 e göre daha kaliteli olduğu için tercih ettim
#define FRAMES_PER_BUFFER (1920)  // standart kullandığım buffer ın boyutu. bunu arttırsanız ses kalitesi artar ama aynı anda gecikme de anlar
#define NUMBEROFCHANNELS (2) // kanal sayısı stereo için bu değer sabit 2 dir
#define PA_SAMPLE_TYPE paFloat32 // sample türü çoğu ses uygulamasında araştırdığım kadarıyla paFloat32 kullanılır.
#define MAX_PACKET_SIZE (1275) //paket paket göndereceğimiz için sıkıştırma ve çözmede kullanılacak paketler max boyutu 
#define PORT 12345 // bilgisayardaki uygulamaların kullandığı portlardan farklı olsun diye kullandığım bir port numarası
#define IP_ADDRESS "127.0.0.1" // her bilgisayar için kendi ip sidir
#define BITRATE 128000 // yüksek ses kalitesi için bit hızının yüksek olması gerekir ama yüksek bit hızı dah büyük paketler gerektirir

typedef float SAMPLE; // bunndan sonra SAMPLE ı kullanacağız.

template <typename T>
class safeQueue {
public:
    void enqueue(const T& value) {
        std::unique_lock<std::mutex> lock(mutex); // queue ya eleman eklemek için mutex i kitler
        thequeue.push(value); // kuyruğa eleman ekler
        conditionV.notify_one(); // bekleyen thread varsa uyandırır 
    }

    T dequeue() {
        std::unique_lock<std::mutex> lock(mutex); // queue dan eleman çıkarmak için mutexi kitle
        while (thequeue.empty()) { // queue boşsa bekle
            conditionV.wait(lock);
        }
        T value = thequeue.front(); // queue daki elemanı ak
        thequeue.pop(); // queuedaki elemanı çıkar
        return value;
    }

private:
    std::queue<T> thequeue; // queue yapısı
    std::mutex mutex; // queue ya erişimi kontrol etmek için bir mutex
    std::condition_variable conditionV; // queue durumunu bildirmek için bir conditional variable.
};

using namespace std;

typedef struct {
    safeQueue<vector<SAMPLE>> queue; // safequeue sınıfı içerir
} paTestData; // ileride kullanacağım data struct ının oluşumu

void checkError(PaError& error) {
    if (error != paNoError) { // hata vaarsa hatanın olduğunu belirtir ve açıklamasını yazar.
        std::cerr << "PortAudio error: " << Pa_GetErrorText(error) << std::endl;
        exit(EXIT_FAILURE);
    }
}

static int recordCallbackFunction(const void* inputBuffer, void* outputBuffer,
    unsigned long framesPerBuffer,
    const PaStreamCallbackTimeInfo* timeInfo,
    PaStreamCallbackFlags status,
    void* userData)
    /*
   *** buradaki status ve timeInfo  kullanmayacağımız verilerdir.

    ***inputBuffer sesin kayıtlı olduğu yerdir biz harici olarak buffer diye bir vector oluşturduk. Doğrudan inputBufferı kullanmamamın amacı ses üzerinde oynama yaptığım --
    için hem orjinal veriyi kayıtlı tutup hem de oynadığım veriyi farklı bir bufferda tutmak istememdi. daha sonra bu buffer'ı ileride alacğaım inputdata nın içindeki queue ya --
    pushladım.
    
    */
{
    paTestData* data = (paTestData*)userData; // inputData
    const SAMPLE* reader = (const SAMPLE*)inputBuffer; // okuyacağımzı değer budur. Ses verisi inputBuffer ın içinde

    vector<SAMPLE> buffer; // oluşturduğumuz geçici tampon. Bunun içine işlem görmüş sesi gömeceğiz
    buffer.reserve(framesPerBuffer * NUMBEROFCHANNELS); // bufferın kapasite ayarı yapılır

    if (inputBuffer == NULL) { // // input buffer'ı boşsa, buffer'ı sıfırla
        for (int i = 0; i < framesPerBuffer * NUMBEROFCHANNELS; i++) {
            buffer.push_back(0.0f);
        }
    }
    else { // boş değilse de bufferı input bufferdaki değerlerle doldur. buradaki 0.8f değeri sesin yüksekliğini ayarlar ve oluşan ses -1 ile 1 arasında değerlenir. Çoğu sesli iletişim kodunda ses verisi flaot cinsinden -1 1 aralığındadır.
        for (int i = 0; i < framesPerBuffer * NUMBEROFCHANNELS; i++) {
            buffer.push_back(std::min(std::max(reader[i] * 0.8f, -1.0f), 1.0f));
        }
    }

    data->queue.enqueue(buffer); // buffer ı queueya ekler.

    return paContinue;
}


static int playCallback(const void* inputBuffer, void* outputBuffer,
    unsigned long framesPerBuffer,
    const PaStreamCallbackTimeInfo* timeInfo,
    PaStreamCallbackFlags status,
    void* userData)
    /*
    *** buradaki status ve timeInfo  kullanmayacağımız verilerdir.

    *** outputBuffer sesin kaydedileceği yerdir. Biz bu sefer queue'daki buffer ı çekiyoruz fonksiyonun içindeki buffer değerine. daha sonra bu bufferdaki sesi çıkış bufferımıza yazıyoruz.

    */

{
    paTestData* data = (paTestData*)userData; //kullanacağımız data ben outputdata diye ayrı bir userData oluşturdum.
    SAMPLE* writer = (SAMPLE*)outputBuffer; // yazılacak yer
    vector<SAMPLE> buffer = data->queue.dequeue(); // queuedan bufferı alırız

    for (int i = 0; i < framesPerBuffer * NUMBEROFCHANNELS; i++) {
        *writer++ = buffer[i];
    }

    return paContinue;
}


void sendAudio(asio::ip::udp::socket& send_socket, asio::ip::udp::endpoint& receiver_endpoint, OpusEncoder* encoder, safeQueue<vector<SAMPLE>>& sendQueue) {
    /*
    temel mantığı sesi iletmektir. Ses verileri sendQueue dan alınır. Alınan veri inputData'nın queue sundan alınır. daha sonra sıkıştırılır ve program sonuna kadar bu işlem receiver_endpointe gönderilir.
    
    
    */

    unsigned char compressedData[MAX_PACKET_SIZE]; // içine sıkıştırılacak datanın boyutunu ayarlar 
    int compressedSize;

    while (true) { // herhangi bir break ifadesi yok yani program bitimine kadar bu fonksiypn çalışacak
        vector<SAMPLE> frame = sendQueue.dequeue(); // queue nun içinden veri alınır

        compressedSize = opus_encode_float(encoder, frame.data(), FRAMES_PER_BUFFER, compressedData, MAX_PACKET_SIZE); // sıkıştırma işleminin gerçekleştiği yer
        if (compressedSize < 0) { // hata kontrolü
            std::cerr << "Opus encoding failed: " << opus_strerror(compressedSize) << std::endl;
            continue; 
        }

        asio::error_code err; // asiodaki hata kontrolü için oluşturuldu
        send_socket.send_to(asio::buffer(compressedData, compressedSize), receiver_endpoint, 0, err); // burda asio aracılığıyla ses yollanır
        if (err) { // sesin hata kontrolü sağlanır
            std::cerr << "Send failed: " << err.message() << std::endl;
        }
    }
}

void receiveAudio(asio::ip::udp::socket& receive_socket, OpusDecoder* decoder, safeQueue<vector<SAMPLE>>& receiveQueue) {

    std::vector<unsigned char> receive_buffer(MAX_PACKET_SIZE);// Alınan veriler için buffer
    /*
  * fonksiyon ses verilerini ağdan sıkıştırılmış şekilde alınır  Opus kullanılarak çözülür ve `receiveQueue` isimli queueya eklenir
  */
    while (true) {
        asio::ip::udp::endpoint remote_endpoint;
        asio::error_code err;
        size_t len = receive_socket.receive_from(asio::buffer(receive_buffer), remote_endpoint, 0, err);
        //alırken oluşan hata tespiti
        if (err && err != asio::error::message_size) {
            std::cerr << "Receive failed: " << err.message() << std::endl;
            continue;
        }
        // çözülecek verileri yerleştirmek için vektör oluştur
        vector<SAMPLE> decodedFrame(FRAMES_PER_BUFFER * NUMBEROFCHANNELS);
        int decodedFrameSize = opus_decode_float(decoder, receive_buffer.data(), len, decodedFrame.data(), FRAMES_PER_BUFFER, 0); // verileri çözer
        if (decodedFrameSize < 0) {
            std::cerr << "Opus decoding failed: " << opus_strerror(decodedFrameSize) << std::endl;
            continue;
        }

        receiveQueue.enqueue(decodedFrame);
    }
}

int main(void) {
  
    PaError err = paNoError;
    OpusEncoder* encoder;
    OpusDecoder* decoder;
    int opusError;

    asio::io_context io_context;
    asio::ip::udp::socket send_socket(io_context);
    asio::ip::udp::socket receive_socket(io_context);
    asio::ip::udp::endpoint receiver_endpoint(asio::ip::make_address(IP_ADDRESS), PORT);
    asio::ip::udp::endpoint sender_endpoint(asio::ip::udp::v4(), PORT);


    send_socket.open(asio::ip::udp::v4());
    receive_socket.open(asio::ip::udp::v4());
    receive_socket.bind(sender_endpoint);

    encoder = opus_encoder_create(SAMPLE_RATE, NUMBEROFCHANNELS, OPUS_APPLICATION_VOIP, &opusError);
    if (opusError != OPUS_OK) {
        std::cerr << "Failed to create Opus encoder: " << opus_strerror(opusError) << std::endl;
        exit(EXIT_FAILURE);
    }

    decoder = opus_decoder_create(SAMPLE_RATE, NUMBEROFCHANNELS, &opusError);
    if (opusError != OPUS_OK) {
        std::cerr << "Failed to create Opus decoder: " << opus_strerror(opusError) << std::endl;
        exit(EXIT_FAILURE);
    }

    
    opus_encoder_ctl(encoder, OPUS_SET_BITRATE(BITRATE));
    opus_encoder_ctl(encoder, OPUS_SET_BANDWIDTH(OPUS_BANDWIDTH_FULLBAND));
    opus_encoder_ctl(encoder, OPUS_SET_COMPLEXITY(10));
    opus_encoder_ctl(encoder, OPUS_SET_SIGNAL(OPUS_SIGNAL_VOICE));
    opus_encoder_ctl(encoder, OPUS_SET_INBAND_FEC(1));
    opus_encoder_ctl(encoder, OPUS_SET_PACKET_LOSS_PERC(10)); 
    opus_encoder_ctl(encoder, OPUS_SET_DTX(1));

    
    err = Pa_Initialize();
    checkError(err);


    PaStream* inputStream;
    paTestData inputData;

    PaStreamParameters inputParameters;

    inputParameters.device = Pa_GetDefaultInputDevice();

    if (inputParameters.device == paNoDevice) {
        std::cerr << "Error: No default input device." << std::endl;
        exit(EXIT_FAILURE);
    }

    inputParameters.channelCount = NUMBEROFCHANNELS;
    inputParameters.sampleFormat = PA_SAMPLE_TYPE;
    inputParameters.suggestedLatency = Pa_GetDeviceInfo(inputParameters.device)->defaultLowInputLatency;

    inputParameters.hostApiSpecificStreamInfo = NULL;

    err = Pa_OpenStream(&inputStream, &inputParameters, NULL, SAMPLE_RATE, FRAMES_PER_BUFFER, paClipOff, recordCallbackFunction, &inputData);
    checkError(err);

    err = Pa_StartStream(inputStream);
    checkError(err);


    PaStream* outputStream;
    paTestData outputData;

    PaStreamParameters outputParameters;
    outputParameters.device = Pa_GetDefaultOutputDevice();
    if (outputParameters.device == paNoDevice) {
        std::cerr << "Error: No default output device." << std::endl;
        exit(EXIT_FAILURE);
    }

    outputParameters.channelCount = NUMBEROFCHANNELS;
    outputParameters.sampleFormat = PA_SAMPLE_TYPE;
    outputParameters.suggestedLatency = Pa_GetDeviceInfo(outputParameters.device)->defaultLowOutputLatency;
    outputParameters.hostApiSpecificStreamInfo = NULL;

    err = Pa_OpenStream(&outputStream, NULL, &outputParameters, SAMPLE_RATE, FRAMES_PER_BUFFER, paClipOff, playCallback, &outputData);
    checkError(err);

    err = Pa_StartStream(outputStream);
    checkError(err);


    thread sendThread(sendAudio, std::ref(send_socket), std::ref(receiver_endpoint), encoder, std::ref(inputData.queue));
    thread receiveThread(receiveAudio, std::ref(receive_socket), decoder, std::ref(outputData.queue));

    sendThread.join();
    receiveThread.join();

    err = Pa_StopStream(inputStream);
    checkError(err);
    err = Pa_CloseStream(inputStream);
    checkError(err);

    err = Pa_StopStream(outputStream);
    checkError(err);
    err = Pa_CloseStream(outputStream);
    checkError(err);

    opus_encoder_destroy(encoder);
    opus_decoder_destroy(decoder);

    Pa_Terminate();
   
    return err;
}

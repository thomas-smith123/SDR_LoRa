#include "board_read.h"

#include <QDebug>

#ifdef simulate
#include "random"
#endif

board_read::board_read(QObject *parent, std::size_t buffer_size)
    : QObject{parent}
{
    board_read::buffer_size = buffer_size;
    start_flag = false;
    // emit_signal_to_process = true;
    config_flag = 1;
    I0 = (int16_t*)malloc(sizeof(int16_t) * buffer_size);
    Q0 = (int16_t*)malloc(sizeof(int16_t) * buffer_size);
    // I1 = (int16_t*)malloc(sizeof(int16_t) * 1024 * 1024);
    // Q1 = (int16_t*)malloc(sizeof(int16_t) * 1024 * 1024);
}
void board_read::config(float bw=5,float fs=10,float lo=1091)
{
    iq_lock = new QMutex;
    config_flag = 1;
    stop_ = true;
    start_flag = 0;
    ctx = NULL;
    rx0_i = NULL;
    rx0_q = NULL;
    tx0_i = NULL;
    tx0_q = NULL;
    rx1_i = NULL;
    rx1_q = NULL;
    tx1_i = NULL;
    tx1_q = NULL;
    rx = NULL;
    tx = NULL;
    rxbuf = NULL;
    txbuf = NULL;
    // RX stream config
    rxcfg.bw_hz = MHZ(bw);   // 2 MHz rf bandwidth
    rxcfg.fs_hz = MHZ(fs);   // 2.5 MS/s rx sample rate
    rxcfg.lo_hz = MHZ(lo); // 2.5 GHz rf frequency
    rxcfg.rfport = "A_BALANCED"; // port A (select for rf freq.)
#ifndef simulate
    if(this->config_flag)
        IIO_ENSURE((ctx = iio_create_context_from_uri(ip)) && "No context","No context");
    if(this->config_flag)
    {
        IIO_ENSURE(iio_context_get_devices_count(ctx) > 0 && "No devices","No devices");

        // printf("* Acquiring AD9361 streaming devices\n");
    }
    //IIO_ENSURE(get_ad9361_stream_dev(ctx, TX, &tx) && "No tx dev found");
    if(this->config_flag)
    {
        IIO_ENSURE(get_ad9361_stream_dev(ctx, RX, &rx) && "No rx dev found","No rx dev found");

        // printf("* Configuring AD9361 for streaming\n");
    }
    if(this->config_flag)
    {
        IIO_ENSURE(cfg_ad9361_streaming_ch(ctx, &rxcfg, RX, 0) && "RX port 0 not found","RX port 0 not found");
        if (!this->config_flag) return;
        // IIO_ENSURE(cfg_ad9361_streaming_ch(ctx, &rxcfg, RX, 1) && "RX port 1 not found","RX port 1 not found");
        //IIO_ENSURE(cfg_ad9361_streaming_ch(ctx, &txcfg, TX, 0) && "TX port 0 not found");

        // printf("* Initializing AD9361 IIO streaming channels\n");
    }
    if(this->config_flag)
    {
        IIO_ENSURE(get_ad9361_stream_ch(ctx, RX, rx, 0, &rx0_i) && "RX chan i not found","RX chan i not found");
        IIO_ENSURE(get_ad9361_stream_ch(ctx, RX, rx, 1, &rx0_q) && "RX chan q not found","RX chan q not found");
        // IIO_ENSURE(get_ad9361_stream_ch(ctx, RX, rx, 2, &rx1_i) && "RX chan i not found","RX chan i not found");
        // IIO_ENSURE(get_ad9361_stream_ch(ctx, RX, rx, 3, &rx1_q) && "RX chan q not found","RX chan q not found");
        // IIO_ENSURE(get_ad9361_stream_ch(ctx, TX, tx, 0, &tx0_i) && "TX chan i not found", "TX chan i not found");
        // IIO_ENSURE(get_ad9361_stream_ch(ctx, TX, tx, 1, &tx0_q) && "TX chan q not found", "TX chan q not found");

        // printf("* Enabling IIO streaming channels\n");

        iio_channel_enable(rx0_i);
        iio_channel_enable(rx0_q);
        // iio_channel_enable(rx1_i);
        // iio_channel_enable(rx1_q);
        // iio_channel_enable(tx0_i);
        // iio_channel_enable(tx0_q);

        // printf("* Creating non-cyclic IIO buffers with 1 MiS\n");
        rxbuf = iio_device_create_buffer(rx, board_read::buffer_size, false);
        // txbuf = iio_device_create_buffer(tx, board_read::buffer_size, false);

    }
    if(this->config_flag)
    {
        if (!rxbuf) {
            perror("Could not create RX buffer");
            shutdownDevice();
            config_flag = 0;
            return;
        }
        p_inc = iio_buffer_step(rxbuf);
        p_end = (char*)iio_buffer_end(rxbuf);
        // if (!txbuf) {
        //     perror("Could not create RX buffer");
        //     shutdownDevice();
        // }
    }
#else

#endif

}
void board_read::start_read()
{
    if(start_flag)
    {
        // ssize_t nbytes_rx;
        // char *p_dat, *p_end;
        // ptrdiff_t p_inc;

        while (!this->stop_)
        {
            // QMutexLocker locker(&sharedresources->mutex);
            // while (sharedresources->isProcessing) {
            //     sharedresources->condition.wait(&sharedresources->mutex);
            //     // if(this->stop_)break;
            // }
            qint64 cnt = 0;
            // if (sharedresources->emit_signal_to_process)
            {
                // Refill RX buffer
                nbytes_rx = iio_buffer_refill(rxbuf);
                if (nbytes_rx < 0) { printf("Error refilling buf %d\n", (int)nbytes_rx); shutdownDevice(); }
                // READ: Get pointers to RX buf and read IQ from RX buf port 0.
                // libiio buffer metadata can change after refill, so refresh it every time.
                p_inc = iio_buffer_step(rxbuf);
                p_end = (char*)iio_buffer_end(rxbuf);
                for (p_dat = (char*)iio_buffer_first(rxbuf, rx0_i); p_dat < p_end; p_dat += p_inc) {
                    // Example: swap I and Q
                    I0[cnt] = ((int16_t*)p_dat)[0];// Real (I)
                    Q0[cnt] = ((int16_t*)p_dat)[1];// Imag (Q)
                    // I1[cnt] = ((int16_t*)p_dat)[2];// Real (I)
                    // Q1[cnt] = ((int16_t*)p_dat)[3];// Imag (Q)
                    cnt++;
                }
                // sharedresources->emit_signal_to_process = false;
                QVector<qint16> iFrame(static_cast<int>(cnt));
                QVector<qint16> qFrame(static_cast<int>(cnt));
                for (qint64 n = 0; n < cnt; ++n) {
                    iFrame[static_cast<int>(n)] = I0[n];
                    qFrame[static_cast<int>(n)] = Q0[n];
                }
                emit iqFrameReady(iFrame, qFrame, static_cast<qint64>(rxcfg.fs_hz));
            }
        }
        qDebug()<<"exit reading";
    }

}
void board_read::reset_to_emmit()
{
    emit_signal_to_process = true;
}
board_read::~board_read()
{
    free(I0); free(Q0);
    shutdownDevice();
}
void board_read::shutdownDevice()
{
    printf("* Destroying buffers\n");
    if (rxbuf) { iio_buffer_destroy(rxbuf); }
    if (txbuf) { iio_buffer_destroy(txbuf); }
    rxbuf = NULL;
    txbuf = NULL;

    printf("* Disabling streaming channels\n");
    if (board_read::rx0_i) { iio_channel_disable(board_read::rx0_i); }
    if (board_read::rx0_q) { iio_channel_disable(board_read::rx0_q); }
    if (board_read::rx1_i) { iio_channel_disable(board_read::rx1_i); }
    if (board_read::rx1_q) { iio_channel_disable(board_read::rx1_q); }
    if (board_read::tx0_i) { iio_channel_disable(board_read::tx0_i); }
    if (board_read::tx0_q) { iio_channel_disable(board_read::tx0_q); }
    rx0_i = NULL;
    rx0_q = NULL;
    rx1_i = NULL;
    rx1_q = NULL;
    tx0_i = NULL;
    tx0_q = NULL;
    tx1_i = NULL;
    tx1_q = NULL;

    printf("* Destroying context\n");
    if (ctx) { iio_context_destroy(ctx); }
    ctx = NULL;
}

/* check return value of attr_write function */
bool board_read::errchk(int v, const char* what) {
    if (v < 0) {
        fprintf(stderr, "Error %d writing to channel \"%s\"\nvalue may not be supported.\n", v, what);
        config_flag = 0;
        stop_ = true;
        shutdownDevice();
        return false;
    }
    return true;
}

/* write attribute: long long int */
bool board_read::wr_ch_lli(struct iio_channel* chn, const char* what, long long val)
{
    return errchk(iio_channel_attr_write_longlong(chn, what, val), what);
}

/* write attribute: string */
bool board_read::wr_ch_str(struct iio_channel* chn, const char* what, const char* str)
{
    return errchk(iio_channel_attr_write(chn, what, str), what);
}

/* helper function generating channel names */
char* board_read::get_ch_name(const char* type, int id)
{
    snprintf(tmpstr, sizeof(tmpstr), "%s%d", type, id);
    return board_read::tmpstr;
}

/* returns ad9361 phy device */
struct iio_device* board_read::get_ad9361_phy(struct iio_context* ctx)
{
    struct iio_device* dev = iio_context_find_device(ctx, "ad9361-phy");
    IIO_ENSURE(dev && "No ad9361-phy found","No ad9361-phy found");
    return dev;
}

/* finds AD9361 streaming IIO devices */
bool board_read::get_ad9361_stream_dev(struct iio_context* ctx, enum iodev d, struct iio_device** dev)
{
    switch (d) {
    case TX: *dev = iio_context_find_device(ctx, "cf-ad9361-dds-core-lpc");
        return *dev != NULL;
    case RX: *dev = iio_context_find_device(ctx, "cf-ad9361-lpc");
        return *dev != NULL;
    default: IIO_ENSURE(0,"get_ad9361_stream_dev false");
        return false;
    }
}
bool board_read::get_ad9361_stream_ch(__notused struct iio_context* ctx, enum iodev d, struct iio_device* dev, int chid, struct iio_channel** chn)
{
    *chn = iio_device_find_channel(dev, get_ch_name("voltage", chid), d == TX);
    if (!*chn)
        *chn = iio_device_find_channel(dev, get_ch_name("altvoltage", chid), d == TX);
    return *chn != NULL;
}

/* finds AD9361 phy IIO configuration channel with id chid */
bool board_read::get_phy_chan(struct iio_context* ctx, enum iodev d, int chid, struct iio_channel** chn)
{
    switch (d) {
    case RX: *chn = iio_device_find_channel(get_ad9361_phy(ctx), get_ch_name("voltage", chid), false); return *chn != NULL;
    case TX: *chn = iio_device_find_channel(get_ad9361_phy(ctx), get_ch_name("voltage", chid), true);  return *chn != NULL;
    default: IIO_ENSURE(0,"get_phy_chan false"); return false;
    }
}

/* finds AD9361 local oscillator IIO configuration channels */
bool board_read::get_lo_chan(struct iio_context* ctx, enum iodev d, struct iio_channel** chn)
{
    switch (d) {
        // LO chan is always output, i.e. true
    case RX: *chn = iio_device_find_channel(get_ad9361_phy(ctx), get_ch_name("altvoltage", 0), true); return *chn != NULL;
    case TX: *chn = iio_device_find_channel(get_ad9361_phy(ctx), get_ch_name("altvoltage", 1), true); return *chn != NULL;
    default: IIO_ENSURE(0,"get_lo_chan false"); return false;
    }
}

/* applies streaming configuration through IIO */
bool board_read::cfg_ad9361_streaming_ch(struct iio_context* ctx, struct stream_cfg* cfg, enum iodev type, int chid)
{
    struct iio_channel* chn = NULL;

    // Configure phy and lo channels
    printf("* Acquiring AD9361 phy channel %d\n", chid);
    if (!get_phy_chan(ctx, type, chid, &chn)) { return false; }
    if (!wr_ch_str(chn, "rf_port_select", cfg->rfport)) return false;
    if (!wr_ch_lli(chn, "rf_bandwidth", cfg->bw_hz)) return false;
    if (!wr_ch_lli(chn, "sampling_frequency", cfg->fs_hz)) return false;

    // Configure LO channel
    printf("* Acquiring AD9361 %s lo channel\n", type == TX ? "TX" : "RX");
    if (!get_lo_chan(ctx, type, &chn)) { return false; }
    if (!wr_ch_lli(chn, "frequency", cfg->lo_hz)) return false;
    return true;
}
void board_read::IIO_ENSURE(int expr, QString reason)
{
    if (!(expr)) { \
            QMessageBox::warning(NULL, "warning", reason, QMessageBox::Ok);\
            board_read::config_flag = 0;
    }
}

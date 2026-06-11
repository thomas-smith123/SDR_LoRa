QT       += core gui printsupport opengl

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

CONFIG += c++17

DESTDIR = $$PWD/bin
OBJECTS_DIR = $$PWD/build/qmake/obj
MOC_DIR = $$PWD/build/qmake/moc
UI_DIR = $$PWD/build/qmake/ui
RCC_DIR = $$PWD/build/qmake/rcc

INCLUDEPATH += \
    $$PWD/include/app \
    $$PWD/include/lora \
    $$PWD/third_party/qcustomplot \
    $$PWD/third_party/libiio \
    $$PWD/third_party/fftw

DEFINES += LORA_USE_FFTW QCUSTOMPLOT_USE_OPENGL

LIBS += -L$$PWD/lib -lfftw3 -liio
QMAKE_RPATHDIR += $$PWD/lib

# You can make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

SOURCES += \
    board_read.cpp \
    src/app/lora_decode_worker.cpp \
    src/app/lora_detector_worker.cpp \
    src/app/spectrum_calc_worker.cpp \
    src/app/spectrum_waterfall_widget.cpp \
    src/app/lora_tx_worker.cpp \
    src/app/main.cpp \
    src/app/mainwindow.cpp \
    src/lora/LoRaPHY.cpp \
    third_party/qcustomplot/qcustomplot.cpp

HEADERS += \
    board_read.h \
    include/app/lora_decode_worker.h \
    include/app/lora_detector_worker.h \
    include/app/spectrum_calc_worker.h \
    include/app/spectrum_waterfall_widget.h \
    include/app/lora_tx_worker.h \
    include/app/mainwindow.h \
    include/lora/LoRaPHY.hpp \
    third_party/qcustomplot/qcustomplot.h

FORMS += \
    ui/mainwindow.ui

DISTFILES += \
    config.ini

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target

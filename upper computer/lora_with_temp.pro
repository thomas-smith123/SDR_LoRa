QT       += core gui printsupport opengl

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

CONFIG += c++17

PROJECT_ROOT = $$_PRO_FILE_PWD_

DESTDIR = "$$PROJECT_ROOT/bin"
OBJECTS_DIR = "$$PROJECT_ROOT/build/qmake/obj"
MOC_DIR = "$$PROJECT_ROOT/build/qmake/moc"
UI_DIR = "$$PROJECT_ROOT/build/qmake/ui"
RCC_DIR = "$$PROJECT_ROOT/build/qmake/rcc"

INCLUDEPATH += \
    "$$PROJECT_ROOT/include/app" \
    "$$PROJECT_ROOT/include/lora" \
    "$$PROJECT_ROOT/third_party/qcustomplot" \
    "$$PROJECT_ROOT/third_party/libiio" \
    "$$PROJECT_ROOT/third_party/fftw"

DEFINES += LORA_USE_FFTW QCUSTOMPLOT_USE_OPENGL

# Platform-specific library configuration
win32 {
    # Windows-specific settings
    FFTW_LIB_DIR = "$$PROJECT_ROOT/third_party/fftw"
    LIBIIO_LIB_DIR = "$$PROJECT_ROOT/third_party/libiio"

    # Debug uses fftw3d.lib, Release uses fftw3.lib
    win32:CONFIG(debug, debug|release) {
        LIBS += "$$FFTW_LIB_DIR/fftw3d.lib"
    } else:win32:CONFIG(release, debug|release) {
        LIBS += "$$FFTW_LIB_DIR/fftw3.lib"
    }
    # libiio: Use full path to libiio.lib (MSVC linker strips 'lib' prefix from -llibiio -> libio.lib which is wrong)
    LIBS += "$$LIBIIO_LIB_DIR/libiio.lib"

    LIBS += opengl32.lib glu32.lib

    # Copy runtime DLLs next to the executable after build.
    QMAKE_POST_LINK += $$quote(powershell -Command "Copy-Item '$$FFTW_LIB_DIR/fftw3.dll' '$$DESTDIR' -Force; Copy-Item '$$LIBIIO_LIB_DIR/libiio.dll' '$$DESTDIR' -Force")
} else:unix {
    # Unix/Linux settings
    LIBS += -L"$$PROJECT_ROOT/lib" -lfftw3 -liio
    QMAKE_RPATHDIR += "$$PROJECT_ROOT/lib"
}

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

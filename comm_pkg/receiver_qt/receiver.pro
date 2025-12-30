QT += widgets
CONFIG += c++17

SOURCES += \
    main.cpp \
    mainwidget.cpp \
    tab7camviewer.cpp \
    mesh_rx_thread.cpp

HEADERS += \
    mainwidget.h \
    tab7camviewer.h \
    mesh_rx_thread.h

FORMS += \
    mainwidget.ui \
    tab7camviewer.ui

INCLUDEPATH += /usr/local/include/opencv4
LIBS += `pkg-config opencv4 --cflags --libs`

RESOURCES += \
    Images.qrc

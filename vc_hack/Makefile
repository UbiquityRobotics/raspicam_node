CC := gcc
AR := ar
RANLIB := ranlib
INSTALL_DIRECTORY := /opt/vc/lib/
DYNAMIC_LIBRARY_BASES :=	\
    libbcm_host			\
    libvcos			\
    libmmal			\
    libmmal_core		\
    libmmal_util		\
    libmmal_vc_client
STATIC_LIBRARY_BASES :=		\
    libvchostif
DYNAMIC_LIBRARIES := ${DYNAMIC_LIBRARY_BASES:%=%.so}
STATIC_LIBRARIES := ${STATIC_LIBRARY_BASES:%=%.a}
ALL_LIBRARIES :=		\
    ${DYNAMIC_LIBRARIES}	\
    ${STATIC_LIBRARIES}

.PHONY: all clean install

all: ${ALL_LIBRARIES}

clean:
	rm -f ${ALL_LIBRARIES}

install: ${ALL_LIBRARIES}
	echo "do something here"
	sudo mkdir -p $(INSTALL_DIRECTORY)
	sudo cp ${ALL_LIBRARIES} $(INSTALL_DIRECTORY)

libbcm_host.so: bcm_host.c
	$(CC) -shared -o $@ -fPIC bcm_host.c

libvcos.so: vcos.c
	$(CC) -shared -o $@ -fPIC vcos.c

libmmal.so: mmal.c
	$(CC) -shared -o $@ -fPIC mmal.c

libmmal_core.so: mmal_core.c
	$(CC) -shared -o $@ -fPIC mmal_core.c

libmmal_util.so: mmal_util.c
	$(CC) -shared -o $@ -fPIC mmal_util.c

libmmal_vc_client.so: mmal_vc_client.c
	$(CC) -shared -o $@ -fPIC mmal_vc_client.c

libvchostif.a: vchostif.c
	$(CC) -c vchostif.c
	$(AR) rcs $@ vchostif.o
	$(RANLIB) $@




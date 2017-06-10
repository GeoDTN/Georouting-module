## sub directory

routing_SOURCES = \
	geoProphetRoutingExtension.cpp \
	geoProphetRoutingExtension.h \
	geoDeliveryPredictabilityMap.h \
	geoDeliveryPredictabilityMap.cpp \
	geoAcknowledgementSet.h \
	geoAcknowledgementSet.cpp \
	geoForwardingStrategy.h \
	geoForwardingStrategy.cpp 
       # gps_read.h \
       # gps_read.cpp
#########################################
#-std=c++0x
#-std=gnu++0x
#CXXFLAGS=-std=c++0x
#-std=gnu++11
#-std=c++11
##############################################
AM_CPPFLAGS = -I$(top_srcdir)/src $(ibrdtn_CFLAGS)
AM_LDFLAGS = $(ibrdtn_LIBS) -lgps

if ANDROID
noinst_DATA = Android.mk
CLEANFILES = Android.mk
else
noinst_LTLIBRARIES = librtgeoprophet.la
librtgeoprophet_la_SOURCES= $(routing_SOURCES)
endif

Android.mk: Makefile.am
	$(ANDROGENIZER) -:PROJECT dtnd \
		-:REL_TOP $(top_srcdir) -:ABS_TOP $(abs_top_srcdir) \
		-:STATIC libdtnd_rtgeoprophet \
		-:SOURCES $(routing_SOURCES) \
		-:CPPFLAGS $(CPPFLAGS) $(AM_CPPFLAGS) \
		-:LDFLAGS $(AM_LDFLAGS) \
		> $@

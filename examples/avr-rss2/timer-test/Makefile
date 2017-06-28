ifndef TARGET
TARGET=avr-rss2
endif

CFLAGS += -DPROJECT_CONF_H=\"project-conf.h\"
CONTIKI_PROJECT += timer-test 

CONTIKI_WITH_RIME = 1
all: $(CONTIKI_PROJECT) 

CONTIKI = ../../../
include $(CONTIKI)/Makefile.include

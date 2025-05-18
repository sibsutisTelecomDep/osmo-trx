#
# Copyright 2008 Free Software Foundation, Inc.
# Copyright 2010 Range Networks, Inc.
#
# This software is distributed under the terms of the GNU Public License.
# See the COPYING file in the main directory for details.
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.
#

include $(top_srcdir)/Makefile.common

SUBDIRS = arch device

AM_CPPFLAGS = -Wall $(STD_DEFINES_AND_INCLUDES) -I${srcdir}/arch/common -I${srcdir}/device/common

AM_CXXFLAGS = -g -O0 -pthread $(LIBOSMOCORE_CFLAGS) $(LIBOSMOCTRL_CFLAGS) $(LIBOSMOVTY_CFLAGS)
AM_CFLAGS = -g -O0 -pthread $(LIBOSMOCORE_CFLAGS) $(LIBOSMOCTRL_CFLAGS) $(LIBOSMOVTY_CFLAGS)

# Базовые AM_LDFLAGS (если есть)
AM_LDFLAGS =

# Переменные для визуализатора (определяются всегда, используются условно)
# ВАЖНО: Установите правильное имя папки вашего сабмодуля
VISUALIZER_SUBMODULE_ACTUAL_NAME = trx-visualizer
VISUALIZER_SRC_PATH = $(top_srcdir)/$(VISUALIZER_SUBMODULE_ACTUAL_NAME)
VISUALIZER_BUILD_SUBDIR_NAME = $(VISUALIZER_SUBMODULE_ACTUAL_NAME)_build
VISUALIZER_BUILD_PATH = $(top_builddir)/$(VISUALIZER_BUILD_SUBDIR_NAME)

VISUALIZER_INCLUDE_PATH = $(VISUALIZER_SRC_PATH)/include
VISUALIZER_EXTERN_IMGUI_PATH = $(VISUALIZER_SRC_PATH)/extern/imgui
VISUALIZER_EXTERN_IMPLOT_PATH = $(VISUALIZER_SRC_PATH)/extern/implot

VISUALIZER_LIB_OUTPUT_PATH = $(VISUALIZER_BUILD_PATH)/lib

# Имя файла API библиотеки, как его генерирует CMake (например, libtrx-visualizer_api.so)
# Убедитесь, что это имя соответствует тому, что генерирует ваш CMake.
VISUALIZER_API_LIB_FILENAME = libtrx_visualizer_api.so
VISUALIZER_API_LIB_FULL_PATH = $(VISUALIZER_LIB_OUTPUT_PATH)/$(VISUALIZER_API_LIB_FILENAME)

RPATH_FOR_VISUALIZER_LIBS = \
    -Wl,-rpath,'$$ORIGIN/../$(VISUALIZER_SUBMODULE_ACTUAL_NAME)_build/lib'

if ENABLE_VISUALIZER
# Добавляем пути к заголовочным файлам визуализатора в глобальные AM_CPPFLAGS
# Макрос WITH_VISUALIZER должен быть определен через config.h
AM_CPPFLAGS += \
	-I$(VISUALIZER_INCLUDE_PATH) \
	-I$(VISUALIZER_EXTERN_IMGUI_PATH) \
	-I$(VISUALIZER_EXTERN_IMPLOT_PATH) \
	@SDL2_CFLAGS@ \
	@GLEW_CFLAGS@

# Добавляем пути поиска библиотек (-L) и RPATH в глобальные AM_LDFLAGS
#AM_LDFLAGS += \
#	-L$(VISUALIZER_LIB_OUTPUT_PATH) \
#	-Wl,-rpath,$(VISUALIZER_LIB_OUTPUT_PATH)

AM_LDFLAGS += \
	-L$(VISUALIZER_LIB_OUTPUT_PATH) \
	$(RPATH_FOR_VISUALIZER_LIBS)

# Библиотеки визуализатора для добавления в _LDADD
# Например, для libtrx-visualizer_api.so -> -ltrx_visualizer_api
VISUALIZER_LINK_LIBS = \
	-ltrx_visualizer_api \
	-ltrx_visualizer_core \
	@SDL2_LIBS@ \
	@GLEW_LIBS@ \
	@OPENGL_LIBS@ \
	-lstdc++fs # Если используется std::filesystem и нужна явная линковка

# Правило для сборки CMake-проекта визуализатора
$(VISUALIZER_API_LIB_FULL_PATH):
	@echo "Building TRX Visualizer (CMake project) in $(VISUALIZER_BUILD_PATH)..."
	mkdir -p $(VISUALIZER_BUILD_PATH)
	mkdir -p $(VISUALIZER_LIB_OUTPUT_PATH) # Убедимся, что и эта директория существует
	cd $(VISUALIZER_BUILD_PATH) && \
	$(CMAKE) $(VISUALIZER_SRC_PATH) \
	    -DCMAKE_BUILD_TYPE=Release \
	    -DBUILD_TEST=OFF \
	    -DCMAKE_CXX_COMPILER="$(CXX)" \
	    -DCMAKE_C_COMPILER="$(CC)" \
	    -DCMAKE_LIBRARY_OUTPUT_DIRECTORY="$(VISUALIZER_LIB_OUTPUT_PATH)" \
	    -DCMAKE_ARCHIVE_OUTPUT_DIRECTORY="$(VISUALIZER_LIB_OUTPUT_PATH)" \
	    -DCMAKE_RUNTIME_OUTPUT_DIRECTORY="$(VISUALIZER_BUILD_PATH)/bin"
	$(MAKE) -C $(VISUALIZER_BUILD_PATH)

# Правило для очистки директории сборки визуализатора
clean-local:
	@echo "Cleaning TRX Visualizer build directory $(VISUALIZER_BUILD_PATH)..."
	rm -rf $(VISUALIZER_BUILD_PATH)

endif # ENABLE_VISUALIZER

noinst_LTLIBRARIES = libtransceiver_common.la

COMMON_SOURCES = \
	radioInterface.cpp \
	radioVector.cpp \
	radioClock.cpp \
	radioBuffer.cpp \
	sigProcLib.cpp \
	signalVector.cpp \
	Transceiver.cpp \
	ChannelizerBase.cpp \
	Channelizer.cpp \
	Synthesis.cpp \
	proto_trxd.c \
	grgsm_vitac/grgsm_vitac.cpp \
	grgsm_vitac/viterbi_detector.cc

libtransceiver_common_la_SOURCES = \
	$(COMMON_SOURCES) \
	Resampler.cpp \
	radioInterfaceResamp.cpp \
	radioInterfaceMulti.cpp \
	metricCollector.cpp
# AM_CPPFLAGS для libtransceiver_common.la уже содержат пути к визуализатору (если включен)
# AM_LDFLAGS для libtransceiver_common.la (если бы она была программой) также содержали бы пути

noinst_HEADERS = \
	Complex.h \
	radioInterface.h \
	radioVector.h \
	radioClock.h \
	radioBuffer.h \
	sigProcLib.h \
	signalVector.h \
	Transceiver.h \
	Resampler.h \
	ChannelizerBase.h \
	Channelizer.h \
	Synthesis.h \
	proto_trxd.h \
	metricCollector.h

COMMON_LDADD = \
	libtransceiver_common.la \
	$(ARCH_LA) \
	$(GSM_LA) \
	$(COMMON_LA) \
	$(FFTWF_LIBS) \
	$(LIBOSMOCORE_LIBS) \
	$(LIBOSMOCTRL_LIBS) \
	$(LIBOSMOVTY_LIBS)

if ENABLE_MS_TRX
# AM_CPPFLAGS уже определены и могут быть дополнены здесь, если нужно
AM_CPPFLAGS += -I$(top_srcdir)/osmocom-bb/src/host/trxcon/include/
TRXCON_LDADD = \
	$(top_builddir)/osmocom-bb/src/host/trxcon/src/.libs/libtrxcon.a \
	$(top_builddir)/osmocom-bb/src/host/trxcon/src/.libs/libl1sched.a \
	$(top_builddir)/osmocom-bb/src/host/trxcon/src/.libs/libl1gprs.a \
	$(LIBOSMOCODING_LIBS)
MS_LOWER_SRC = \
	ms/sch.c \
	ms/ms.cpp \
	ms/threadsched.cpp \
	ms/ms_rx_lower.cpp \
	grgsm_vitac/grgsm_vitac.cpp \
	grgsm_vitac/viterbi_detector.cc
MS_UPPER_SRC = \
	ms/ms_upper.cpp \
	ms/l1ctl_server.c \
	ms/logging.c \
	ms/l1ctl_server_cb.cpp \
	ms/ms_trxcon_if.cpp
noinst_HEADERS += \
   	ms/ms.h \
	ms/threadsched.h \
	ms/bladerf_specific.h \
	ms/uhd_specific.h \
	ms/ms_upper.h \
	ms/ms_trxcon_if.h \
	ms/itrq.h \
	ms/sch.h \
	ms/threadpool.h \
	grgsm_vitac/viterbi_detector.h \
	grgsm_vitac/constants.h \
	grgsm_vitac/grgsm_vitac.h
endif # ENABLE_MS_TRX

bin_PROGRAMS =

# --- UHD Device ---
if DEVICE_UHD
bin_PROGRAMS += osmo-trx-uhd
osmo_trx_uhd_SOURCES = osmo-trx.cpp
# _CPPFLAGS: Глобальные AM_CPPFLAGS (которые уже включают пути к визуализатору, если ENABLE_VISUALIZER)
# плюс специфичные для UHD.
osmo_trx_uhd_CPPFLAGS = $(AM_CPPFLAGS) $(UHD_CFLAGS)
# _LDFLAGS: Для программ эта переменная обычно не используется напрямую для путей -L,
# Automake использует AM_LDFLAGS. Если нужны специфичные LDFLAGS только для этой цели,
# то osmo_trx_uhd_LDFLAGS = $(что_то_еще). Флаги из AM_LDFLAGS будут добавлены.
# osmo_trx_uhd_LDFLAGS =
# _LDADD: Основные зависимости программы
osmo_trx_uhd_LDADD = \
	$(builddir)/device/uhd/libdevice.la \
	$(COMMON_LDADD) \
	$(UHD_LIBS)
if ENABLE_VISUALIZER
osmo_trx_uhd_LDADD += $(VISUALIZER_LINK_LIBS)
osmo_trx_uhd_DEPENDENCIES = $(VISUALIZER_API_LIB_FULL_PATH)
endif
endif # DEVICE_UHD

# --- USRP1 Device ---
if DEVICE_USRP1
bin_PROGRAMS += osmo-trx-usrp1
osmo_trx_usrp1_SOURCES = osmo-trx.cpp
osmo_trx_usrp1_CPPFLAGS = $(AM_CPPFLAGS) $(USRP_CFLAGS)
osmo_trx_usrp1_LDADD = \
	$(builddir)/device/usrp1/libdevice.la \
	$(COMMON_LDADD) \
	$(USRP_LIBS)
if ENABLE_VISUALIZER
osmo_trx_usrp1_LDADD += $(VISUALIZER_LINK_LIBS)
osmo_trx_usrp1_DEPENDENCIES = $(VISUALIZER_API_LIB_FULL_PATH)
endif
endif # DEVICE_USRP1

# --- LMS Device ---
if DEVICE_LMS
bin_PROGRAMS += osmo-trx-lms
osmo_trx_lms_SOURCES = osmo-trx.cpp
osmo_trx_lms_CPPFLAGS = $(AM_CPPFLAGS) $(LMS_CFLAGS)
osmo_trx_lms_LDADD = \
	$(builddir)/device/lms/libdevice.la \
	$(COMMON_LDADD) \
	$(LMS_LIBS)
if ENABLE_VISUALIZER
osmo_trx_lms_LDADD += $(VISUALIZER_LINK_LIBS)
osmo_trx_lms_DEPENDENCIES = $(VISUALIZER_API_LIB_FULL_PATH)
endif
endif # DEVICE_LMS

# --- BladeRF Device ---
if DEVICE_BLADE
bin_PROGRAMS += osmo-trx-blade
osmo_trx_blade_SOURCES = osmo-trx.cpp
# Оригинал: osmo_trx_blade_CPPFLAGS  = $(AM_CPPFLAGS) $(LMS_CFLAGS) - возможно, здесь должно быть $(BLADE_CFLAGS)?
osmo_trx_blade_CPPFLAGS = $(AM_CPPFLAGS) $(BLADE_CFLAGS) # Заменено на BLADE_CFLAGS для консистентности
osmo_trx_blade_LDADD = \
	$(builddir)/device/bladerf/libdevice.la \
	$(COMMON_LDADD) \
	$(BLADE_LIBS)
if ENABLE_VISUALIZER
osmo_trx_blade_LDADD += $(VISUALIZER_LINK_LIBS)
osmo_trx_blade_DEPENDENCIES = $(VISUALIZER_API_LIB_FULL_PATH)
endif

if ENABLE_MS_TRX
bin_PROGRAMS += osmo-trx-ms-blade
osmo_trx_ms_blade_SOURCES = $(MS_LOWER_SRC) $(MS_UPPER_SRC)
osmo_trx_ms_blade_CPPFLAGS = $(AM_CPPFLAGS) $(BLADE_CFLAGS) -DBUILDBLADE
osmo_trx_ms_blade_LDADD = \
	$(builddir)/device/bladerf/libdevice.la \
	$(COMMON_LDADD) \
	$(BLADE_LIBS) \
	$(TRXCON_LDADD)
if ENABLE_VISUALIZER
osmo_trx_ms_blade_LDADD += $(VISUALIZER_LINK_LIBS)
osmo_trx_ms_blade_DEPENDENCIES = $(VISUALIZER_API_LIB_FULL_PATH)
endif
endif # ENABLE_MS_TRX for blade
endif # DEVICE_BLADE

# --- IPC Device ---
if DEVICE_IPC
bin_PROGRAMS += osmo-trx-ipc
osmo_trx_ipc_SOURCES = osmo-trx.cpp
osmo_trx_ipc_CPPFLAGS = $(AM_CPPFLAGS)
osmo_trx_ipc_LDADD = \
	$(builddir)/device/ipc/libdevice.la \
	$(COMMON_LDADD)
if ENABLE_VISUALIZER
osmo_trx_ipc_LDADD += $(VISUALIZER_LINK_LIBS)
osmo_trx_ipc_DEPENDENCIES = $(VISUALIZER_API_LIB_FULL_PATH)
endif
endif # DEVICE_IPC
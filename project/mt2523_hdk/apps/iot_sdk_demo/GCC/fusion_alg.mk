
FUSIONALG   = project/mt2523_hdk/apps/iot_sdk_demo/src/fusion_algorithm

C_FILES  += $(FUSIONALG)/fusion_alg_interface_api.c    \
            $(FUSIONALG)/fusion_alg_manager.c		   \
            $(FUSIONALG)/SendGps.c					   \
            $(FUSIONALG)/BasicFunc.c				   \
            $(FUSIONALG)/DataProc.c  				   \
            $(FUSIONALG)/GetAcc.c   				   \
            $(FUSIONALG)/GIFilter.c   				   \
            $(FUSIONALG)/GINavMain.c   				   \
            $(FUSIONALG)/GlobalVars.c  				   \
            $(FUSIONALG)/Gnss.c  			     	   \
            $(FUSIONALG)/InsAlign.c  			       \
            $(FUSIONALG)/InsNav.c  			           \
            


CFLAGS  += -I$(SOURCE_DIR)/project/mt2523_hdk/apps/iot_sdk_demo/inc/fusion_algorithm

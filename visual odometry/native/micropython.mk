# Pass the parent directory as USER_C_MODULES for Make-based ports.
RM_VO_MOD_DIR := $(USERMOD_DIR)
SRC_USERMOD += $(RM_VO_MOD_DIR)/modvo.c
SRC_USERMOD += $(RM_VO_MOD_DIR)/vo_core.c
CFLAGS_USERMOD += -I$(RM_VO_MOD_DIR) -std=c99

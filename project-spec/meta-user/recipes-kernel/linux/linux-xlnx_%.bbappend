FILESEXTRAPATHS:prepend := "${THISDIR}/${PN}:"

SRC_URI:append = " file://bsp.cfg"
KERNEL_FEATURES:append = " bsp.cfg"
SRC_URI += "file://user_2025-10-13-10-21-00.cfg \
            file://user_2025-10-15-07-45-00.cfg \
            "


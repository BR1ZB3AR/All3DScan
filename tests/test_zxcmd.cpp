#include <catch2/catch_test_macros.hpp>

#include "net/ZxCmdProtocol.h"

using namespace revoscan::net;

TEST_CASE("zxCmdUrl builds cgi URLs with and without custom port") {
    ZxHost host;
    host.host = "192.168.3.1";
    REQUIRE(zxCmdUrl(host, zxGetSerial()) ==
            "http://192.168.3.1/cgi-bin/zx_cmd.cgi?get_sn");

    host.port = 8080;
    REQUIRE(zxCmdUrl(host, "get_sn") ==
            "http://192.168.3.1:8080/cgi-bin/zx_cmd.cgi?get_sn");
}

TEST_CASE("zx command builders match observed RevoScan query strings") {
    REQUIRE(zxGetExposure() == "cam_type=usb&get_exposure");
    REQUIRE(zxSetExposure(3000) == "cam_type=usb&set_exposure=3000");
    REQUIRE(zxSetExposure(-5) == "cam_type=usb&set_exposure=0");
    REQUIRE(zxGetGain() == "cam_type=usb&get_gain");
    REQUIRE(zxGetExposureRange() == "cam_type=usb&get_exposureRange");
    REQUIRE(zxSetAeMaxExposure(120) == "cam_type=usb&set_aeMaxExp=120");
    REQUIRE(zxGetFarNearState() == "get_farNearState");
    REQUIRE(zxSetRedLightIndicator(true) == "set_RedLightIndicator=1");
    REQUIRE(zxSetRedLightIndicator(false) == "set_RedLightIndicator=0");
}

TEST_CASE("parseIntResponse extracts integers from varied bodies") {
    REQUIRE(parseIntResponse("3000").value() == 3000);
    REQUIRE(parseIntResponse("  exposure=2500 \n").value() == 2500);
    REQUIRE(parseIntResponse("{\"exposure\":1800}").value() == 1800);
    REQUIRE(parseIntResponse("value: -42").value() == -42);
    REQUIRE_FALSE(parseIntResponse("no number here").has_value());
    REQUIRE_FALSE(parseIntResponse("").has_value());
}

TEST_CASE("parseSerialResponse pulls a serial token") {
    REQUIRE(parseSerialResponse("D2654557").value() == "D2654557");
    REQUIRE(parseSerialResponse("  D2654557  \n").value() == "D2654557");
    REQUIRE(parseSerialResponse("{\"sn\":\"D2654557\"}").value() == "D2654557");
    REQUIRE_FALSE(parseSerialResponse("   ").has_value());
}

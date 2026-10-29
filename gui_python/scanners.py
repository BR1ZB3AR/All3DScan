"""Which 3D scanners are plugged in, and what this app can do with each.

Driving a scanner live needs that model's USB protocol, which no vendor
publishes and which has to be recovered with the hardware in hand. Reading
what a scanner exported needs nothing of the sort. So a scanner this cannot
drive is still worth finding and naming: the window can say what it is, say
plainly that it cannot scan from it yet, and point at File -> Open scan,
which works for every scanner there is.

The USB ids come from resources/devices.json, the same list the Qt app
matches against, so a model added there is found here too.
"""

from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path

from common import HERE

DEVICES_JSON = HERE.parent / "resources" / "devices.json"
SYSFS_USB = Path("/sys/bus/usb/devices")

#: The scanners this app can take a scan from, by USB id. Everything else
#: that is recognised is named and explained, not offered as a scanner.
#: A model only joins this list once scanning has been tested on it.
DRIVABLE = {
    ("2207", "110c"): "Revopoint POP 4",
}


@dataclass(frozen=True)
class Scanner:
    """One scanner found on the bus."""

    vendor_id: str
    product_id: str
    vendor: str
    label: str

    @property
    def key(self) -> str:
        return f"{self.vendor_id}:{self.product_id}"

    @property
    def drivable(self) -> bool:
        return (self.vendor_id, self.product_id) in DRIVABLE

    @property
    def name(self) -> str:
        """What to call it in the scanner list."""
        if self.drivable:
            return f"{DRIVABLE[(self.vendor_id, self.product_id)]} (USB)"
        return f"{self.vendor} {self.product_id} — cannot scan yet"

    def why_not(self) -> str:
        """What can be done with a scanner this cannot drive."""
        return (
            f"{self.vendor} {self.key} is plugged in, and All3DScan recognises it, but "
            f"nobody has worked out how to take a scan from this model on Linux yet — "
            f"that needs its USB protocol reverse-engineered with the scanner in hand.\n\n"
            f"What does work today: scan with the vendor's own software, export the "
            f"result, and open it here with File → Open scan. Cleaning, meshing, "
            f"fitting planes and cylinders and exporting STEP all work on it.\n\n"
            f"If you would like to help get this model scanning natively, the hardware "
            f"support issue template asks for the captures it would take."
        )


def known_devices(path: Path | None = None) -> list[dict]:
    """The scanner ids the project knows, from resources/devices.json.

    A broken or missing file leaves the app working with no list rather than
    failing to start: it only costs the naming of other scanners.
    """
    try:
        listed = json.loads((path or DEVICES_JSON).read_text())["devices"]
    except (OSError, ValueError, KeyError):
        return []
    return [entry for entry in listed if entry.get("enabled", True)]


def usb_devices(sysfs: Path | None = None) -> list[tuple[str, str]]:
    """Every USB device's (vendor id, product id), lower case, from sysfs."""
    found = []
    try:
        entries = sorted((sysfs or SYSFS_USB).iterdir())
    except OSError:
        return found
    for device in entries:
        try:
            vendor = (device / "idVendor").read_text().strip().lower()
            product = (device / "idProduct").read_text().strip().lower()
        except OSError:
            continue  # a hub, an interface, or one that went away as we looked
        if vendor and product:
            found.append((vendor, product))
    return found


def plugged_in(sysfs: Path | None = None, devices: Path | None = None) -> list[Scanner]:
    """The scanners on the bus, the ones this can drive first.

    A device matches an entry when the vendor agrees and either the product
    agrees or the entry covers the whole vendor, which is how devices.json
    says "any model from these people".
    """
    known = known_devices(devices)
    found: dict[tuple[str, str], Scanner] = {}
    for vendor_id, product_id in usb_devices(sysfs):
        for entry in known:
            wanted = str(entry.get("vendorId", "")).lower()
            product = str(entry.get("productId", "*")).lower()
            if wanted != vendor_id or product not in ("*", "", product_id):
                continue
            scanner = Scanner(vendor_id, product_id,
                              entry.get("vendor", "Unknown"),
                              entry.get("label", ""))
            # An entry naming this exact product beats a vendor-wide one.
            if product != "*" or (vendor_id, product_id) not in found:
                found[(vendor_id, product_id)] = scanner
            break
    return sorted(found.values(), key=lambda s: (not s.drivable, s.vendor, s.product_id))

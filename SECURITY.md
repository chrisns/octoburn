# Security

OctoBurn drives a Class 4 laser. Please report anything that could make it move or fire when it
should not, or that exposes the printer, privately through GitHub's "Report a vulnerability"
(Security tab), not in a public issue.

These gaps are by design. The README covers them under "Access and authentication":

- The LightBurn bridge (GRBL over TCP, port 2323) has no authentication or encryption, because
  LightBurn cannot send credentials. It accepts only the addresses in `bridge_allow`. Firewall
  it to the LightBurn computer and tunnel it over SSH for anything beyond the LAN.
- The close-up camera images are served without an API key, because LightBurn's network camera
  cannot send one.
- The probe board's ESPHome web server has no authentication. Keep it on a trusted network.

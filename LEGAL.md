# Legal notice

MicroScribe Inspector — © 2026 Marconnet Technologies. Source code under the [MIT license](LICENSE);
documentation and images under [CC BY 4.0](LICENSE-docs).

This file explains how the project is put together so that users and contributors can judge it for
themselves. **It is not legal advice.**

## No affiliation

MicroScribe Inspector is an independent open-source project by Marconnet Technologies. It is not
affiliated with, authorised, endorsed or supported by the owner of the MicroScribe® trademark or by
any company that made or sold MicroScribe products. "MicroScribe" and other product names are
trademarks of their respective owners and are used only to say which hardware this software works
with.

## What the repository contains

Only source code, documentation, images and build scripts written for this project. It contains no
third-party or manufacturer software, firmware, manuals or other manufacturer material.

## Where the protocol knowledge comes from

The serial protocol and the position calculations are documented in
[docs/PROTOCOL.md](docs/PROTOCOL.md), with the source of each statement:

* the MIT-licensed open-source [microcmm](https://github.com/Revise-Robotics/microcmm) project,
* the open-source [VRPN](https://github.com/vrpn/vrpn) project, and
* observation of the replies of an arm owned by the authors, using read-only commands.

The purpose is **interoperability**: letting owners keep using hardware they own.

## How the software treats the hardware

* It does not bypass any licence check, copy protection or access control.
* It never sends the commands that change the arm's stored calibration (SET_PARAMS, SET_HOME_REF,
  RESTORE_FACTORY, SET_HOME). The only state-changing command, HOME_POS (re-zeroing the encoders in
  the home pose), is sent only when the user explicitly asks for it.

## Contributions

By contributing you confirm that your contribution is your own work (or compatibly licensed) and
contains no manufacturer software, firmware, manuals or material copied from them. Protocol findings
are welcome when they come from open-source projects or from observing a device you own; please say
which in the pull request.

## No warranty

The software is provided "as is", without warranty of any kind (see [LICENSE](LICENSE)). Measuring
results depend on the arm, its calibration and how it is used — verify critical dimensions
independently.

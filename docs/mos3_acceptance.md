# ngspice MOS Level-3 acceptance contract

## Reference implementation

The behavioural reference for this work is the default `MOS3` implementation
of **ngspice 46**.  Reference listings are generated locally with the exact
`ngspice` executable recorded in `tests/cases/mos3/SOURCES.md`; they are never
generated with this simulator.

The target is the normal ngspice MOS3 algorithm, including the corrected
`KAPPA` behaviour used by modern ngspice.  The historical algorithm selected by
ngspice's `badmos3` option is explicitly out of scope.  A deck which requests
that option must not silently select the normal algorithm.

## Milestone scope

The first implementation milestone is numerical agreement for the analyses
this project already provides:

| Area | Required for MOS3 milestone |
| --- | --- |
| Analysis | `.op` and `.tran` only |
| Terminals | Four terminal D/G/S/B device, including bulk effects |
| DC model | MOS3 channel current, `gm`, `gds`, `gmb`, B-D and B-S diodes |
| Geometry | `L`, `W`, `M`, `AD`, `AS`, `PD`, `PS`, `NRD`, `NRS`, `LD`, `XL`, `XW`, `WD` |
| Series resistance | `RD`/`RS` and `RSH*NRD/NRS`, using D-prime/S-prime internal nodes |
| Charge | overlap, Meyer gate charge, and B-D/B-S depletion charge in transient analysis |
| Model inputs | all MOS3 DC/charge inputs listed in the ngspice MOS3 parameter table |
| Reference comparison | committed ngspice 46 `.out` listings plus the MOS3 suite below |

`TEMP`, `TNOM`, `OFF`, and instance `IC=VDS,VGS,VBS` belong to the complete
MOS3 netlist contract, but cannot be claimed until the simulator has a
circuit-wide temperature path and non-zero transient initial conditions.  They
are deliberately excluded from the first executable fixtures rather than
accepted and ignored.

AC and noise analyses are outside this milestone because this simulator does
not implement `.ac` or `.noise`.  `KF` and `AF` may be parsed and preserved for
netlist compatibility, but have no observable effect until noise analysis is
added.

## Parameters and compatibility rules

The implementation must distinguish MOS3 from the project's Level-1 model at
model-build time.  It must not reuse Level-1's `LAMBDA` square-law equation for
a `LEVEL=3` card.  MOS3 accepts these model inputs when relevant to its
implemented analysis:

`VTO/VT0`, `KP`, `GAMMA`, `PHI`, `RD`, `RS`, `CBD`, `CBS`, `IS`, `PB`,
`CGSO`, `CGDO`, `CGBO`, `RSH`, `CJ`, `MJ`, `CJSW`, `MJSW`, `JS`, `TOX`,
`LD`, `XL`, `XW`, `WD`, `UO/U0`, `FC`, `NSUB`, `TPG`, `NSS`, `VMAX`, `XJ`,
`NFS`, `ETA`, `DELTA`, `THETA`, `KAPPA`, `TNOM`, `KF`, and `AF`.

MOS3 instance cards must support `M`, `L`, `W`, `AD`, `AS`, `PD`, `PS`,
`NRD`, `NRS`, `OFF`, `IC`, and `TEMP`.  Explicit model inputs override
process-derived values, as in ngspice.

`GMIN` and `RDS` are existing simulator-specific MOS conveniences, not ngspice
MOS3 model-card parameters.  Strict MOS3 mode must reject them or move their
meaning to a documented solver-level option; it must never reinterpret them as
`RD`/`RS`.

## Numerical acceptance

The current executable subset is the complete DC path: channel current,
internal D-prime/S-prime topology, source/drain resistance, and B-D/B-S
junction currents. `make test-mos3-dc` compares the four corresponding
official OP fixtures with ngspice 46. The MOS3 unit target is part of `make
test`; the complete `make test-mos3` gate remains deferred until overlap and
Meyer gate-charge modelling is implemented.

The initial comparison policy is configurable from the Makefile and defaults
to `atol=1e-7`, `rtol=2e-3` for OP and transient output values.  Tighten these
after the first independently reproduced MOS3 implementation; do not relax
them merely to preserve a Level-1 approximation.

Every fixture must state its upstream source and transformation in
`tests/cases/mos3/SOURCES.md`.  The unmodified upstream inputs remain
identifiable by URL, immutable revision or ticket identifier, and SHA-256.

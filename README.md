# robot-chassis

3D-printable robot chassis designs written as Python CAD code with
[CADbuildr](https://cadbuildr.com) foundation.

- **`RobotChassis`** — a single-deck 2WD chassis: Uno R4, 2 x TT gearmotors,
  6xAA pack, printed ball castor and HC-SR04 mount.
- **`MebotBase`** — a 3D-printable take on the Makeblock mBot base, resized to a
  120 x 100 mm top plate with every mBot hole pattern kept on the 8/16 mm grid.

## View the models in 3D

```bash
uv sync
pip install cadbuildr-broker
cadbuildr-broker            # leave running (port 5050)
```

Open [hub.cadbuildr.com/viewer](https://hub.cadbuildr.com/viewer), then:

```bash
uv run python starter/robot_chassis.py
uv run python starter/mebot_base.py
```

On Windows, set `PYTHONUTF8=1` first, otherwise `show(...)` fails printing its
status message (`'charmap' codec can't encode character`).

## What's in here

```
├── starter/
│   ├── robot_chassis.py        2WD chassis: printed parts + reference components
│   ├── mebot_base.py           MEBOT base (mBot-style) part
│   └── mebot_base_data.py      generated outlines/hole data for mebot_base.py
├── make_base_data.py       regenerates mebot_base_data.py from the mBot STEP file
├── mebot_base_V1.3mf       print-ready MEBOT base
├── pyproject.toml          Python package manifest
└── logo.png                used by CADbuildr Prototype's project picker
```

See the docstring at the top of each model file for print orientation,
hardware and dimensions to measure before printing.

## Make it your own

1. **Rename the package.** Change `starter/` to your package name; mirror
   it in `pyproject.toml` (`[project].name`, `[tool.hatch.build.targets.wheel].packages`).
2. **Write your Parts.** A Part is a Python class that adds operations
   (sketches, extrusions, fillets, etc.) to itself. See
   [`mebot_base.py`](starter/mebot_base.py) for an example.
3. **Assemble them.** An Assembly adds Parts (or other Assemblies) with
   transforms. See [`robot_chassis.py`](starter/robot_chassis.py).
4. **Ship your own demo site.** Copy `github-io/`, change the package
   imports + the wheel filename in `src/starterLocal.ts`, set
   `VITE_APP_BASE_PATH` to your repo path, and let the
   [GitHub Pages workflow](.github/workflows/deploy-pages.yml) do the rest.

## Deploy your own GitHub Pages demo (optional)

The site uses the [CADbuildr SDK browser flow](https://docs.cadbuildr.com/sdk/auth) — a
short-lived session token is minted client-side using only the
**publishable half** of a partner key (the secret stays on hub).

1. Mint an SDK key at https://hub.cadbuildr.com/settings:
   - **Allowed origins**: `https://<your-github-username>.github.io`
   - **Allowed projects**: `starter-template`
2. In your repo's *Settings → Secrets and variables → Actions*, add a
   secret named **`CADBUILDR_SDK_KEY_ID`** with the public `<keyId>` half
   of your key (e.g. `624fe53d82c9fa7c`, not the full `cbsdk_*` string).
3. Enable Pages with build source = "GitHub Actions" and push.

The workflow rebuilds the Python wheel, stages it into
`github-io/public/local-starter/`, then builds and deploys the Vite site.

## License

MIT — fork freely.

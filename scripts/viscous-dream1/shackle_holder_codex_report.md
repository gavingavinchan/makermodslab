Created and verified the **21 × 40 × 15.75 mm** holder. The cradle gap is **0.501–0.504 mm**, with no interference, clear pin access, and one watertight body.

- Print: [STL](/home/gavin/dreamtrial/viscous_sim/assets/shackle/shackle_holder.stl) · [3MF](/home/gavin/dreamtrial/viscous_sim/assets/shackle/shackle_holder.3mf)
- [Report](/home/gavin/dreamtrial/viscous_sim/runs/shackle_holder/shackle_holder_report.md) · [Preview](/home/gavin/dreamtrial/viscous_sim/runs/shackle_holder/holder_with_shackle_3q.png) · [Parametric script](/home/gavin/dreamtrial/viscous_sim/shackle_holder.py)
- Verified 1:1 PDF: :codex-file-citation{path="/home/gavin/dreamtrial/viscous_sim/runs/shackle_holder/side_view_holder.pdf" purpose="output"}

**The glue is structural:** with the pin seated, the 20 N push requires about **42 N saddle hold-down** and **40 N table hold-down**. The 2 N·m torque estimates **2.22 MPa peak saddle-bond shear**. Joint strength remains unverified; the optional second-block sketch is included.

No billed calls, hardware, or sim-server use. Recorded source files remained unchanged.

- :codex-followup[Fit real measurements]{prompt="Update the cradle using real-part measurements I provide."}
- :codex-followup[Build optional block]{prompt="Generate the optional second support while preserving pin clearance."}
- :codex-followup[Add to simulation]{prompt="Add the holder to a local MuJoCo scene without billed calls, episodes, or hardware."}
# Attribution

- **Z-Anatomy** — https://github.com/Z-Anatomy/The-blend — licensed CC BY-SA 4.0.
- **BodyParts3D**, © The Database Center for Life Science, CC BY 4.0 (mirror: https://github.com/Kevin-Mattheus-Moerman/BodyParts3D).
- **HDRI lighting** (`assets/env/`) — Poly Haven, https://polyhaven.com, CC0.
- **Carpal kinematics** (`assets/carpal_model.json`) — coefficients copied from Supplementary Table S1 of Akhbari B, Moore DC, Laidlaw DH, Weiss A-PC, Akelman E, Wolfe SW, Crisco JJ. *Predicting carpal bone kinematics using an expanded digital database of wrist carpal bone anatomy and kinematics.* J Orthop Res 2019;37(12):2661–2670, doi:10.1002/jor.24435. Open database: https://simtk.org/projects/carpal-database.
- **Tendon moment arms** (`tools/arms_ma.json`, the `mech` tables in `assets/cables.json`) — computed with OpenSim 4.6 from the ARMS hand-and-wrist model: McFarland DC, Binder-Markey BI, Nichols JA, Wohlman SJ, de Bruin M, Murray WM. *A musculoskeletal model of the hand and wrist capable of simulating functional tasks.* IEEE Trans Biomed Eng 2023;70(5):1424–1435, doi:10.1109/TBME.2022.3217722 (model on https://simtk.org). Its finger paths build on An KN et al., J Biomech 1983;16:419 and Holzbaur KR et al., Ann Biomed Eng 2005;33:829. Only derived moment-arm values are included here, not the model files.
- Tissue detail maps (`assets/tex/*.ktx2`) are generated procedurally by `tools/make_tissue_tex.py`.

The 3D assets in this repository (`assets/`) are derived from the above and are licensed **CC BY-SA 4.0**.

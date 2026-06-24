# Journal Bearing Reynolds Solver and Worn Bearing Dynamics

End-to-end computational pipeline from the Reynolds equation governing lubricant pressure to the rotor vibration signature of a worn journal bearing. Implements an eight-stage workflow: 2-D finite-difference Reynolds solver with Swift–Stieber cavitation, eight linearised dynamic coefficients by perturbation, Jeffcott-rotor stability with oil-whirl threshold and whirl-to-whip transition visible, Dufrane worn-bearing dynamics, unbalance response showing the −1× backward synchronous signature, and least-squares wear identification from a simulated vibration measurement.

**Author:** Akshat Verma

---

## Scope at a glance

**Part A — Reynolds solver.**

1. **Ocvirk short-bearing** analytical solution and the Sommerfeld number $S = (\mu N / P)(R/C)^2$. Closed-form pressure field, equilibrium curve, attitude angle.
2. **2-D finite-difference Reynolds solver.** Sparse CSR with vectorised COO assembly; ~35 ms per Gümbel solve, ~80 ms per Swift–Stieber solve at 240×41. Cross-validated against Ocvirk at $L/D = 0.25$.
3. **Eight linearised dynamic coefficients** $K_{ij}, C_{ij}$ via central finite-difference perturbation. Step-size convergence demonstrated; analytical Ocvirk-perturbation cross-check at the same equilibrium.
4. **Stability threshold and oil whirl / whip onset.** Dual-load analysis: at design load (W = 5000 N, S = 0.25) the bearing is robustly stable across its operating envelope; at reduced load (W = 800 N, S = 1.56) the threshold appears at 7813 rpm with whirl ratio 0.526 (classical half-frequency whirl confirmed), and the whirl-to-whip transition becomes visible as Im(λ)/Ω falls from 0.53 to 0.31 over the post-threshold sweep.

**Part B — Worn bearing dynamics.**

5. **Dufrane two-parameter wear model** $\delta h(\theta) = \max[0,\, d_0 - C(1 + \cos(\theta - \gamma))]$.
6. **Worn-bearing coefficients.** Sweep wear depth $d_0$, recompute $K_{ij}, C_{ij}$, observe anisotropy growth.
7. **Unbalance response** showing the −1× backward synchronous signature emerge monotonically with wear depth (Alves et al. 2022 signature reproduced).
8. **Wear identification.** Nelder–Mead least-squares inversion of $(d_0, \gamma)$ from simulated +1× and −1× amplitudes.

---

## How to run

```bash
pip install numpy scipy matplotlib jupyter
jupyter notebook Project4_Bearing_Reynolds.ipynb
```

**Parameter changes.** All physical constants live in a single `BEARING` dictionary at the top of the notebook (Stage 1). Substituting a different bearing geometry — e.g. one of the tilting-pad bearings on the TRC test rig, or any in-service refinery bearing — is a one-line change. The grid resolution (`n_theta`, `n_z`) is also there; the wear sweep and stability sweep use a coarser 180×31 grid for speed, while Stage 1–3 use 240×41 for precision.

---

## Key numerical results

At the design parameters ($R = 50$ mm, $L = 50$ mm, $C = 100~\mu$m, $\mu = 0.02$ Pa·s, $\Omega = 314$ rad/s, $W = 5000$ N → $S = 0.25$):

- **Equilibrium eccentricity** $\varepsilon = 0.663$ (FD with Reynolds cavitation).
- **Peak film pressure** 3.06 MPa at $\theta \approx 151°$ (about 30° upstream of $h_\mathrm{min}$).
- **Dimensionless dynamic coefficients** (Gümbel cavitation, $K^* = KC/W$, $C^* = CC\Omega/W$):
  $$K^* \approx \begin{pmatrix} +4.54 & +1.11 \\ -3.07 & +1.03 \end{pmatrix}, \quad C^* \approx \begin{pmatrix} +7.09 & -2.05 \\ -2.73 & +2.21 \end{pmatrix}$$
- **Stability at design load:** robustly stable across the full operating envelope (500–6000 rpm); max Re(λ) reaches only −43 at 6000 rpm. Instability margin is smallest at high speed.
- **Stability at reduced load** (W = 800 N): threshold at **7813 rpm** with whirl ratio **0.526** — within ~5% of the classical half-frequency-whirl prediction (0.47–0.50). The whirl-to-whip transition is visible above threshold as Im(λ)/Ω falls from 0.53 to 0.31 across the post-threshold sweep up to 25,000 rpm.
- **−1× backward synchronous orbit component** grows monotonically with wear depth: 4.30 → 5.98 µm (+39 %) from unworn to $d_0 = C$.
- **Wear identification** from two scalar measurements recovers $d_0$ within ~10 % of truth and $\gamma$ within a few degrees.

---

## Validation

Three independent benchmarks:

1. **Closed-form Ocvirk vs direct 2-D numerical quadrature.** Forces and attitude angle agree to machine precision (10⁻¹⁵).
2. **FD solver vs Ocvirk at $L/D = 0.25$.** 3 % error at $\varepsilon = 0.2$; grows to ~21 % at $\varepsilon = 0.8$ (sharp pressure peak resolution).
3. **FD-to-Ocvirk load ratio across $L/D$.** Ratio 0.97 → 0.45 → 0.26 at $\varepsilon = 0.6$, $L/D = 0.25 \to 1.0$. Matches the canonical short-bearing-breakdown trend reported in Childs (1993) §4.5 and Lund (1987).

Plus a sign-and-magnitude consistency check on all eight dynamic coefficients (cross-coupled $K_{xy} K_{yx} < 0$, diagonal $K_{xx}, K_{yy} > 0$, etc.), and a step-size convergence study confirming a clean plateau in $K^*_{xx}$ across half a decade of perturbation steps.

---

## Important Findings:

1. **Cavitation BC choice affects the perturbation method.** Swift–Stieber (Reynolds) cavitation is an iterative active-set scheme; as the journal position is perturbed by $\delta q$, the active-set boundary jumps cell-by-cell, making $F_x(X)$ piecewise-constant at the sub-cell scale. Central differences then divide a discontinuous quantity by $\delta q$ and blow up. *Verified:* $K^*_{xx}$ bounced between 4.6 and 6.7 with no plateau across step sizes under Reynolds cavitation. **Switching the perturbation method to Gümbel cavitation** (whose boundary is a smooth level set) gave a clean plateau at $K^*_{xx} = 4.54$ across half a decade of step sizes. Reynolds remains the right choice for the equilibrium pressure visualisation in Stage 2; the perturbation method just needs a smoother cavitation model. The observation is consistent with the analytical-perturbation-vs-finite-difference comparison published by Rowe & Chong (1986) and Jang & Lee (2006).

2. **Stability is load-dependent, and the textbook 0.5Ω whirl signature emerges only at threshold.** At the design load (W = 5000 N) the bearing is robustly stable and the Im(λ) curve sits above 0.5Ω — that's a *damped natural frequency*, not a whirl frequency. Calling it "whirl frequency" is a labelling trap that obscures what's actually happening. Only when load is reduced to W = 800 N does the threshold enter the operating speed range, and only then does Im(λ) cross the 0.5Ω line to deliver the classical half-frequency whirl signature. The dual-case analysis in Stage 4 makes this load-dependence explicit.

3. **The rigid-rotor Jeffcott model can show whirl cleanly but not whip cleanly.** Oil whip is the lock-on of the whirl frequency to a fixed natural frequency — in real machines, the shaft bending mode. Our model has no shaft flexibility; the only available natural frequency is the bearing-dominated mode sqrt(K_yy/m), which itself drifts with speed via the Sommerfeld dependence. So we see the *whirl-to-whip transition* (Im(λ) decoupling from 0.5Ω and bending toward a softer trajectory) but not the *flat-plateau whip signature* a flexible-rotor model would produce. Acknowledging the scope honestly is more useful than overclaiming what the model can show. Extending to a flexible-shaft rotor is a natural next step.

4. **The two-amplitude wear inverse problem has a shallow cost valley.** With only +1× and −1× amplitudes at one speed, the optimiser recovers $d_0$ within ~10 % of truth — not severe ill-posedness, but not pinned down either. The natural extension is multi-speed measurement: amplitudes at $N$ speeds give $2N$ constraints for two unknowns, and the valley closes up. This is exactly the kind of inverse-problem refinement that TRC's experimental rigs would support.

---

## References

- Reynolds, O. (1886). On the theory of lubrication. *Phil. Trans. R. Soc.* **177**, 157.
- Ocvirk, F. W. (1952). Short-bearing approximation for full journal bearings. *NACA TN-2808*.
- Dufrane, K. F., Kannel, J. W., McCloskey, T. H. (1983). Wear of steam turbine journal bearings at low operating speeds. *J. Lubr. Tech.* **105**, 187.
- Rowe, W. B., Chong, F. S. (1986). Computation of dynamic force coefficients for hybrid journal bearings by the finite disturbance and perturbation techniques. *Tribology International* **19**(5), 260–271.
- Lund, J. W. (1987). Review of the concept of dynamic coefficients for fluid film journal bearings. *J. Tribol.* **109**, 37–41.
- Childs, D. W. (1993). *Turbomachinery Rotordynamics*. Wiley.
- Jang, G. H., Lee, S. H. (2006). Determination of the dynamic coefficients of the coupled journal and thrust bearings by perturbation method. *Tribology Letters* **22**(3), 239–246.
- Alves, D. S., Wu, M. F., Cavalca, K. L. (2022). Application of asymmetric wear bearings in rotating machinery. *J. Sound Vib.* **524**, 116772.

---

## Contact

Akshat Verma — HPCL Mumbai Refinery — preparing PhD application for Fall 2026.

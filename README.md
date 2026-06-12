# Object Tracking in Videos — MATLAB

*CMPEN 454 — Fundamentals of Computer Vision · The Pennsylvania State University · 2022*

Tracks a moving object across video frames by finding where a bounding box from one frame best matches the next. Two trackers are implemented — one that handles simple translation, and a more sophisticated one that handles objects that rotate, scale, or shear as they move. The result is a yellow bounding box that follows the car across the full video sequence.

Implemented in MATLAB from scratch using the Lucas-Kanade (LK) and Matthews-Baker Inverse Compositional (IC-LK) algorithms. LK iteratively estimates the (u, v) pixel translation that minimises the intensity difference between a template patch and the current frame using Gauss-Newton updates. The Matthews-Baker variant extends this to a full 6-parameter affine warp and precomputes the Jacobian and Hessian inverse on the template once, making each tracking iteration significantly cheaper since only the image warp is recomputed per frame.

**[Live Demo →](https://halkhoori2000.github.io/Tracking-Objects-in-Videos/)**

📄 **[Project Report](docs/project-report.docx)** — the original write-up (method, derivations, results)

---

## Use Cases
- Autonomous vehicle perception: tracking other cars, pedestrians, and obstacles across consecutive camera frames
- Sports analytics: following athletes or equipment across broadcast footage for performance analysis
- Surveillance and security: persistent tracking of objects or individuals through a scene
- Robotics: visual servoing where a robot must keep a target object centred in its field of view

## Challenges
- **Drift accumulation**: each frame's tracking error compounds into the next — a small misalignment in frame N shifts the search region in frame N+1, causing the bounding box to gradually drift off the target over long sequences
- **Gauss-Newton convergence**: the linearised brightness constancy assumption only holds for small displacements; if the object moves too far between frames, the gradient-descent update diverges rather than converging to the correct alignment
- **Affine warp inversion in IC-LK**: the inverse compositional update requires composing W = W · W(Δp)⁻¹ rather than adding Δp directly — an incorrect update rule silently produces wrong warp estimates that still appear numerically stable
- **Precomputation correctness**: the Jacobian J and Hessian H⁻¹ in `initAffineMBTracker` are computed once on the template and reused every frame — any error in this precomputation propagates to every subsequent update without any per-frame signal to detect it

---

## Algorithms

### Lucas-Kanade (Forward Additive)
```
Given: template T (from frame t), current image I (frame t+1), bounding box rect
Initialise: u = 0, v = 0

Repeat until |Δp| < ε:
  1. Warp I to aligned coords:  I_w(x) = I(x + [u, v])
  2. Compute image gradients:   [Ix, Iy] at warped coords
  3. Compute steepest descent:  A = [Ix, Iy] · dW/dp   (dW/dp = I for translation)
  4. Compute update:            Δp = (AᵀA)⁻¹ Aᵀ (T − I_w)
  5. Update warp:               [u, v] += [Δu, Δv]
```

### Matthews-Baker Inverse Compositional (Affine)
```
Precompute once on template T:
  J = ∇T · dW/dp   (Jacobian, 6-parameter affine)
  H⁻¹ = (JᵀJ)⁻¹

Per frame (cheap):
  1. Warp current image:  I_w = I(W(x; p))
  2. Compute update:      Δp = H⁻¹ Jᵀ (I_w − T)
  3. Compose warp:        W ← W · W(Δp)⁻¹
```

---

## Tech Stack

| Item | Detail |
|---|---|
| Language | MATLAB |
| Algorithms | Lucas-Kanade (translation), Matthews-Baker IC-LK (affine) |
| Libraries | None — all tracker logic implemented from scratch |
| Input | Car video sequence (frame-by-frame JPEGs) |
| Output | Tracked bounding box overlaid on video, saved as AVI |

---

## Project Structure

```
Tracking-Objects-in-Videos/
├── src/
│   ├── LucasKanade.m          ← LK forward-additive translation tracker
│   ├── affineMBTracker.m      ← Matthews-Baker IC-LK affine tracker (per frame)
│   ├── initAffineMBTracker.m  ← precomputes J and H⁻¹ from template
│   ├── warpH.m                ← projective image warp utility
│   ├── lk_demo.m              ← LK demo: tracks car, saves AVI
│   └── mb_demo.m              ← MB demo: tracks car, saves AVI
├── results/
│   ├── car.avi                ← original car video
│   ├── car_mb_1.avi           ← LK tracking result (raw)
│   ├── car_mb_2.avi           ← MB tracking result (raw)
│   ├── car.mp4                ← original (web)
│   ├── lk_tracking.mp4        ← LK result (web)
│   └── mb_tracking.mp4        ← MB result (web)
└── index.html                 ← live demo (GitHub Pages)
```

---

## Run

**Requirements:** MATLAB. Place car sequence frames as `data/car/frame0020.jpg` … `frame0280.jpg`.

```matlab
cd src
lk_demo    % Lucas-Kanade tracker → results/car_mb_1.avi
mb_demo    % Matthews-Baker tracker → results/car_mb_2.avi
```

---

## Course

CMPEN 454 — Fundamentals of Computer Vision  
The Pennsylvania State University · 2022

"""
3D viewer for your reconstruction.

    python3 view_viser.py ../captures/<run>              # bundle adjustment, step by step
    python3 view_viser.py ../captures/<run> --two-view   # two-view result from Section 5

Then go to http://localhost:8080 in a browser. Click on a camera to look through it.
"""
import argparse
import os
import threading
import time

import cv2
import numpy as np
import viser
import viser.uplot as uplot
from scipy.spatial.transform import Rotation

from common import run_output_dir

ACCENT = (47, 75, 216)   # cameras
START = (150, 158, 170)  # where the cameras started (SLAM)
MOTION = (217, 130, 0)   # lines from start to current


def wxyz(R):
    x, y, z, w = Rotation.from_matrix(R).as_quat()
    return np.array([w, x, y, z])


def thumbnail(run_dir, name, w, h):
    img = cv2.imread(os.path.join(str(run_dir), str(name)))
    if img is None:
        return None
    return cv2.resize(img, (w // 4, h // 4))[:, :, ::-1]


def show_two_view(out_dir, run_dir, port):
    path = os.path.join(out_dir, 'two_view.npz')
    if not os.path.exists(path):
        raise SystemExit(f'no {path} yet, run two_view.py first')
    d = np.load(path)
    K, P = d['K'], d['points']
    h, w = 480, 640
    server = viser.ViserServer(port=port)
    server.scene.set_up_direction('+z')
    fov = 2 * np.arctan2(h / 2, K[1, 1])
    for k in range(2):
        server.scene.add_camera_frustum(
            f'/cams/{k}', fov=fov, aspect=w / h, scale=0.05, thickness=0.0025, color=ACCENT,
            image=thumbnail(run_dir, d['names'][k], w, h), wxyz=wxyz(d['R_map_cam'][k]),
            position=d['centers'][k])
        server.scene.add_label(f'/cams/{k}/label', str(d['names'][k]), position=(0, 0, -0.012))
    size = server.gui.add_slider('Point size (mm)', min=1.0, max=20.0, step=0.5, initial_value=5.0)
    server.gui.add_markdown(f'### Two-view reconstruction\n{len(P)} points triangulated from '
                            f'{d["names"][0]} and {d["names"][1]}, scaled to meters with the SLAM baseline.')

    def draw(_=None):
        server.scene.add_point_cloud('/points', P, d['colors'].astype(np.uint8),
                                     point_size=size.value / 1000, point_shape='circle')
    draw()
    size.on_update(draw)
    target = np.median(P, axis=0)

    @server.on_client_connect
    def _(client):
        client.camera.position = d['centers'].mean(axis=0) - 0.3 * (target - d['centers'].mean(axis=0)) + [0, 0, 0.3]
        client.camera.look_at = target
        client.camera.up_direction = (0.0, 0.0, 1.0)

    print(f'go to http://localhost:{port} (ctrl-c to quit)')
    while True:
        time.sleep(1.0)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('run_dir')
    ap.add_argument('--two-view', action='store_true', help='show two_view.npz instead of bundle adjustment')
    ap.add_argument('--port', type=int, default=8080)
    args = ap.parse_args()
    out_dir = run_output_dir(args.run_dir)
    if args.two_view:
        return show_two_view(out_dir, args.run_dir, args.port)

    path = os.path.join(out_dir, 'ba_history.npz')
    if not os.path.exists(path):
        raise SystemExit(f'no {path} yet, run multiview.py and bundle_adjust.py first')
    hist = np.load(path)
    n_it, n_cams = len(hist['cost']), hist['R'].shape[1]
    colors = hist['colors'].astype(np.uint8)
    # camera -> world rotations and camera centers for every iteration
    R_wc = np.transpose(hist['R'], (0, 1, 3, 2))
    C = -np.einsum('kcji,kcj->kci', hist['R'], hist['t'])
    X = hist['X']
    loop_center = C[0, :, :2].mean(axis=0)
    radial = np.linalg.norm(X[0, :, :2] - loop_center, axis=1)
    near = radial < 0.2
    floor_z = float(np.percentile(X[0, near, 2], 2)) if near.any() else float(C[0, :, 2].min())
    h, w = 480, 640

    thumbs = []
    for name in hist['names']:
        thumbs.append(thumbnail(args.run_dir, name, w, h))

    server = viser.ViserServer(port=args.port)
    server.scene.set_up_direction('+z')
    server.scene.add_grid('/floor', width=3.0, height=3.0, cell_size=0.1, section_size=0.5,
                          position=(loop_center[0], loop_center[1], floor_z))

    server.gui.add_markdown('### Bundle adjustment\nIteration 0 is your multiview.py result. After that, '
                            'each Levenberg-Marquardt step moves all the cameras and points at once.')
    status = server.gui.add_markdown('')
    it_slider = server.gui.add_slider('Iteration', min=0, max=n_it - 1, step=1, initial_value=0)
    play = server.gui.add_button('Play')
    speed = server.gui.add_dropdown('Step every', options=('0.25 s', '0.5 s', '1 s'), initial_value='0.5 s')
    exag = server.gui.add_slider('Exaggerate motion', min=1, max=30, step=1, initial_value=10)
    server.gui.add_uplot(
        data=(np.arange(n_it, dtype=float), hist['rms'].astype(float)),
        series=(uplot.Series(label='iteration'),
                uplot.Series(label='RMS px', stroke='#2f4bd8', width=2)),
        title='Reprojection RMS (px)', aspect=1.6,
        scales={'x': uplot.Scale(time=False, auto=True), 'y': uplot.Scale(auto=True)})
    with server.gui.add_folder('Display'):
        show_start = server.gui.add_checkbox('SLAM start poses', initial_value=True)
        show_lines = server.gui.add_checkbox('Motion lines', initial_value=True)
        show_imgs = server.gui.add_checkbox('Images on frustums', initial_value=True)
        crop = server.gui.add_slider('Point radius (m)', min=0.1, max=6.0, step=0.05, initial_value=6.0)
        size = server.gui.add_slider('Point size (mm)', min=1.0, max=20.0, step=0.5, initial_value=10.0)
        objects_btn = server.gui.add_button('Look at the objects')
        overview_btn = server.gui.add_button('Overview')
    server.gui.add_markdown('Click a camera to look through it. The real changes are only a few '
                            'centimeters, so the exaggerate slider scales them up so you can see them.')

    fov0 = 2 * np.arctan2(h / 2, hist['f'][0])
    start_frusta = [server.scene.add_camera_frustum(
        f'/start/{c:02d}', fov=fov0, aspect=w / h, scale=0.05, thickness=0.0015,
        color=START, wxyz=wxyz(R_wc[0, c]), position=C[0, c]) for c in range(n_cams)]
    frusta = []
    for c in range(n_cams):
        fr = server.scene.add_camera_frustum(
            f'/cams/{c:02d}', fov=fov0, aspect=w / h, scale=0.05, thickness=0.0025,
            color=ACCENT, image=thumbs[c], wxyz=wxyz(R_wc[0, c]), position=C[0, c])
        fr.on_click(lambda event, c=c: look_through(event.client, c))
        server.scene.add_label(f'/cams/{c:02d}/label', str(c), position=(0, 0, -0.012))
        frusta.append(fr)
    cloud = server.scene.add_point_cloud('/points', X[0], colors, point_size=size.value / 1000,
                                         point_shape='circle')
    state = {'lines': None, 'C': C[0], 'R': R_wc[0]}

    def exaggerated(k):
        e = exag.value
        Ck = C[0] + e * (C[k] - C[0])
        # scale up how much each camera has turned since iteration 0
        d = Rotation.from_matrix(np.einsum('cij,ckj->cik', R_wc[k], R_wc[0])).as_rotvec()
        Rk = np.einsum('cij,cjk->cik', Rotation.from_rotvec(e * d).as_matrix(), R_wc[0])
        Xk = X[0] + e * (X[k] - X[0])
        return Ck, Rk, Xk

    def update(_=None):
        k = int(it_slider.value)
        Ck, Rk, Xk = exaggerated(k)
        state['C'], state['R'] = Ck, Rk
        fov = 2 * np.arctan2(h / 2, hist['f'][k])
        for c, fr in enumerate(frusta):
            fr.position = Ck[c]
            fr.wxyz = wxyz(Rk[c])
            fr.fov = fov
        sel = radial < crop.value
        cloud.points = Xk[sel].astype(np.float32)
        cloud.colors = colors[sel]
        cloud.point_size = size.value / 1000
        if state['lines'] is not None:
            state['lines'].remove()
            state['lines'] = None
        if show_lines.value and k > 0:
            state['lines'] = server.scene.add_line_segments(
                '/motion', np.stack([C[0], Ck], axis=1), colors=MOTION, line_width=2.0)
        status.content = (f'**Iteration {k} of {n_it - 1}**  \n'
                          f'RMS **{hist["rms"][k]:.3f} px** (median {hist["median"][k]:.3f})  \n'
                          f'cost {hist["cost"][k]:.1f}, focal length {hist["f"][k]:.1f} px  \n'
                          f'cameras have moved {np.median(np.linalg.norm(C[k] - C[0], axis=1)) * 1000:.0f} mm '
                          f'(median), shown {exag.value}x bigger')

    def toggle_start(_=None):
        for fr in start_frusta:
            fr.visible = show_start.value

    def toggle_images(_=None):
        for c, fr in enumerate(frusta):
            fr.image = thumbs[c] if show_imgs.value else None

    for handle in (it_slider, exag, crop, size, show_lines):
        handle.on_update(update)
    show_start.on_update(toggle_start)
    show_imgs.on_update(toggle_images)

    def look_through(client, c):
        client.camera.position = state['C'][c]
        client.camera.look_at = state['C'][c] + state['R'][c][:, 2] * 0.4
        client.camera.up_direction = (0.0, 0.0, 1.0)

    target = np.array([loop_center[0], loop_center[1], floor_z + 0.06])

    def set_view(client, eye):
        client.camera.position = eye
        client.camera.look_at = target
        client.camera.up_direction = (0.0, 0.0, 1.0)

    objects_btn.on_click(lambda e: (setattr(crop, 'value', 0.35), set_view(e.client, target + [-0.45, -0.45, 0.35])))
    overview_btn.on_click(lambda e: (setattr(crop, 'value', 6.0), set_view(e.client, target + [-1.4, -1.4, 1.2])))
    server.initial_camera.position = target + [-0.9, -0.9, 0.7]
    server.initial_camera.look_at = target
    server.on_client_connect(lambda client: set_view(client, target + [-0.9, -0.9, 0.7]))

    playing = threading.Event()

    @play.on_click
    def _(_):
        if playing.is_set():
            playing.clear()
            play.label = 'Play'
        else:
            if it_slider.value >= n_it - 1:
                it_slider.value = 0
            playing.set()
            play.label = 'Pause'

    update()
    print(f'go to http://localhost:{args.port} (ctrl-c to quit)')
    while True:
        if playing.is_set():
            if it_slider.value >= n_it - 1:
                playing.clear()
                play.label = 'Play'
            else:
                it_slider.value = it_slider.value + 1  # calls update()
        time.sleep({'0.25 s': 0.25, '0.5 s': 0.5, '1 s': 1.0}[speed.value])


if __name__ == '__main__':
    main()

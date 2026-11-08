// Copyright 2026 MakerMods. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
import { useCallback, useEffect, useMemo, useRef, useState } from "react";
import { useTranslation } from "react-i18next";
import { Button } from "@/components/ui/button";
import {
  Dialog,
  DialogContent,
  DialogDescription,
  DialogHeader,
  DialogTitle,
} from "@/components/ui/dialog";
import { useApi } from "@/contexts/ApiContext";
import { ApiError } from "@/lib/apiClient";
import {
  movedSinceCapture,
  tankApi,
  type TankFace,
  type TankFit,
  type TankStatus,
} from "@/lib/tankApi";

const STEPS = [
  "right",
  "right",
  "left",
  "left",
  "near",
  "near",
  "floor",
  "check",
];
const STEP_KEYS = [
  "rightFirst",
  "rightSecond",
  "leftFirst",
  "leftSecond",
  "nearFirst",
  "nearSecond",
  "floorStep",
  "checkStep",
] as const;
const COLORS: Record<string, string> = {
  near: "#f59e0b",
  far: "#94a3b8",
  left: "#22c55e",
  right: "#38bdf8",
  floor: "#c084fc",
};
const CHAIN = [
  "base_link",
  "yaw_link",
  "upper_arm_link",
  "forearm_link",
  "wrist_pitch_link",
  "wrist_yaw_link",
  "wrist_roll_link",
  "tool0",
];
const WALLS: TankFace[] = ["near", "left", "right"];
const TOWER_LABELS = [
  "tower_front",
  "tower_top",
  "tower_left",
  "tower_right",
] as const;
const fmt = (n: number | null | undefined) => (n == null ? "—" : n.toFixed(1));

function TankDrawing({ status }: { status: TankStatus | null }) {
  const { t } = useTranslation();
  const face = (f: string) =>
    t(`tank.${f === "tower" ? "towerFace" : f}` as never, { defaultValue: f });
  // Base x decreases to screen right, base y increases upward.
  const xy = (p: number[]) => [90 - p[0] * 0.52, 190 - p[1] * 0.52];
  const fit = status?.fit;
  const corners: number[][] = [];
  if (fit?.offsets_mm && fit.yaw_deg != null) {
    const a = (fit.yaw_deg * Math.PI) / 180,
      c = Math.cos(a),
      s = Math.sin(a),
      o = fit.offsets_mm;
    for (const [x, y] of [
      [o.near, o.right],
      [o.far, o.right],
      [o.far, o.left],
      [o.near, o.left],
    ])
      corners.push(xy([c * x - s * y, s * x + c * y]));
  }
  const arm = CHAIN.map((n) => status?.link_origins_mm[n])
    .filter((p): p is number[] => !!p)
    .map((p) => xy(p).join(","))
    .join(" ");
  const tower = status?.tower_box;
  const towerA = tower ? xy(tower.min_mm) : null,
    towerB = tower ? xy(tower.max_mm) : null;
  return (
    <svg
      viewBox="0 0 570 380"
      role="img"
      aria-label={t("tank.drawing")}
      className="w-full rounded-lg border bg-muted/20"
    >
      <title>{t("tank.drawing")}</title>
      <text x="12" y="22" fill="currentColor" fontSize="12">
        {t("tank.axes")}
      </text>
      {!corners.length && (
        <g fill="currentColor" fontSize="11" opacity=".6">
          <text x="230" y="64">
            {face("left")}
          </text>
          <text x="230" y="330">
            {face("right")}
          </text>
          <text x="12" y="160">
            {face("near")}
          </text>
        </g>
      )}
      {corners.map((p, i) => {
        const q = corners[(i + 1) % 4];
        const f = ["right", "far", "left", "near"][i];
        return (
          <g key={f}>
            <line
              x1={p[0]}
              y1={p[1]}
              x2={q[0]}
              y2={q[1]}
              stroke={COLORS[f]}
              strokeWidth="2"
              strokeDasharray={f === "far" ? "7 5" : undefined}
            />
            <text
              x={(p[0] + q[0]) / 2 + 4}
              y={(p[1] + q[1]) / 2 - 8}
              fill={COLORS[f]}
              fontSize="11"
            >
              {face(f)}
              {f === "far" ? ` · ${t("tank.estimated")}` : ""}
            </text>
          </g>
        );
      })}
      {towerA && towerB && (
        <rect
          x={Math.min(towerA[0], towerB[0])}
          y={Math.min(towerA[1], towerB[1])}
          width={Math.abs(towerA[0] - towerB[0])}
          height={Math.abs(towerA[1] - towerB[1])}
          fill="#64748b"
          fillOpacity=".45"
        >
          <title>{t("tank.tower")}</title>
        </rect>
      )}
      <polyline
        points={arm}
        stroke="currentColor"
        strokeWidth="4"
        fill="none"
        opacity=".6"
      />
      <circle cx="90" cy="190" r="7" fill="currentColor" />
      <text x="24" y="211" fill="currentColor" fontSize="11">
        {t("tank.base")}
      </text>
      {status?.touches.map((touch, i) => {
        const p = xy(touch.point_mm);
        return (
          <circle
            key={i}
            cx={p[0]}
            cy={p[1]}
            r="4"
            fill={COLORS[touch.label.replace("check:", "")] ?? "#e879f9"}
          >
            <title>{touch.label}</title>
          </circle>
        );
      })}
      {status?.probe_mm && (
        <circle
          cx={xy(status.probe_mm)[0]}
          cy={xy(status.probe_mm)[1]}
          r="6"
          fill="none"
          stroke="currentColor"
          strokeWidth="2"
        />
      )}
    </svg>
  );
}

/** Daily glass placement, plus once-per-mount tower and cable measurements.
 * Torque stays off; the saved evidence does not constrain teleop yet. */
export default function TankMeasureDialog({
  open,
  onOpenChange,
  robotName,
}: {
  open: boolean;
  onOpenChange: (open: boolean) => void;
  robotName: string;
}) {
  const { t } = useTranslation();
  const { baseUrl, fetchWithHeaders } = useApi();
  const api = useMemo(
    () => tankApi(baseUrl, fetchWithHeaders),
    [baseUrl, fetchWithHeaders],
  );
  const [status, setStatus] = useState<TankStatus | null>(null);
  const [previous, setPrevious] = useState<string | null>(null);
  const [error, setError] = useState<string | null>(null);
  const [saved, setSaved] = useState(false);
  const [busy, setBusy] = useState(false);
  const [auto, setAuto] = useState(true);
  const [step, setStep] = useState(0);
  const [checkFace, setCheckFace] = useState<TankFace>("near");
  const [extraTarget, setExtraTarget] = useState<string | null>(null);
  const [link, setLink] = useState("yaw_link");
  const [envelopeFace, setEnvelopeFace] = useState<TankFace>("near");
  const captureBusy = useRef(false);
  const lastCapture = useRef<number[] | null>(null);
  const moved = useRef(true);
  const audio = useRef<AudioContext | null>(null);
  const active = !!status?.active && status.robot_name === robotName;
  const face = useCallback(
    (f: string) =>
      t(`tank.${f === "tower" ? "towerFace" : f}` as never, {
        defaultValue: f,
      }),
    [t],
  );
  const label = useCallback(
    (s: string): string => {
      if (s.startsWith("check:"))
        return t("tank.checkLabel", { face: face(s.slice(6)) });
      if (s.startsWith("envelope:")) {
        const [, l, f] = s.split(":");
        return t("tank.envelopeLabel", { link: l, face: face(f) });
      }
      return face(s);
    },
    [t, face],
  );
  const target =
    extraTarget ??
    (step < STEPS.length
      ? STEPS[step] === "check"
        ? `check:${checkFace}`
        : STEPS[step]
      : null);
  const report = useCallback(
    (e: unknown) =>
      setError(
        e instanceof ApiError ? (e.detail ?? t("tank.error")) : t("tank.error"),
      ),
    [t],
  );

  useEffect(() => {
    if (!open) return;
    let cancelled = false;
    let timer: ReturnType<typeof setTimeout>;
    setStatus(null);
    setSaved(false);
    setError(null);
    void api
      .latest(robotName)
      .then((r) => {
        if (!cancelled) setPrevious(r.measurement?.measured.timestamp ?? null);
      })
      .catch(report);
    const poll = async () => {
      try {
        const next = await api.status();
        if (!cancelled && (!next.robot_name || next.robot_name === robotName))
          setStatus(next);
      } catch (e) {
        if (!cancelled) {
          report(e);
          setStatus((s) => (s ? { ...s, stale: true, still: false } : s));
        }
      }
      if (!cancelled) timer = setTimeout(poll, 100);
    };
    void poll();
    return () => {
      cancelled = true;
      clearTimeout(timer);
    };
  }, [open, robotName, api, report]);

  const capture = useCallback(async () => {
    if (!target || captureBusy.current) return;
    captureBusy.current = true;
    setBusy(true);
    setError(null);
    setSaved(false);
    try {
      const next = await api.capture(target);
      setStatus(next);
      lastCapture.current = next.touches.at(-1)?.point_mm ?? next.probe_mm;
      moved.current = false;
      if (!extraTarget) setStep((s) => Math.min(s + 1, STEPS.length));
      try {
        if (audio.current) {
          const ctx = audio.current,
            oscillator = ctx.createOscillator(),
            gain = ctx.createGain();
          oscillator.connect(gain);
          gain.connect(ctx.destination);
          gain.gain.value = 0.08;
          oscillator.frequency.value = 880;
          oscillator.start();
          oscillator.stop(ctx.currentTime + 0.1);
        }
      } catch {
        /* A blocked audio device must not undo a captured touch. */
      }
    } catch (e) {
      report(e);
      setAuto(false);
    } finally {
      captureBusy.current = false;
      setBusy(false);
    }
  }, [api, target, extraTarget, report]);

  useEffect(() => {
    if (!open || !active || !status) return;
    if (movedSinceCapture(lastCapture.current, status.probe_mm))
      moved.current = true;
    if (
      auto &&
      moved.current &&
      target &&
      status.still &&
      !status.stale &&
      !busy &&
      (target.startsWith("envelope:") ||
        (status.gripper_opening_mm ?? 100) <= 5)
    )
      void capture();
  }, [open, active, auto, status, busy, target, capture]);

  const start = async () => {
    setBusy(true);
    setError(null);
    setSaved(false);
    try {
      // The operator's click unlocks audio before automatic captures occur.
      try {
        audio.current ??= new AudioContext();
        void audio.current.resume();
      } catch {
        /* Visual feedback still works without audio. */
      }
      const next = await api.start(robotName);
      setStatus(next);
      setStep(0);
      setExtraTarget(null);
      lastCapture.current = null;
      moved.current = true;
    } catch (e) {
      report(e);
    } finally {
      setBusy(false);
    }
  };
  const stop = async () => {
    setBusy(true);
    setError(null);
    try {
      setStatus(await api.stop());
    } catch (e) {
      report(e);
    } finally {
      setBusy(false);
    }
  };
  const close = async () => {
    if (active) {
      setBusy(true);
      try {
        const next = await api.stop();
        setStatus(next);
        if (next.active) return;
      } catch (e) {
        report(e);
        return;
      } finally {
        setBusy(false);
      }
    }
    onOpenChange(false);
  };
  const save = async () => {
    setBusy(true);
    setError(null);
    try {
      const next = await api.save();
      setPrevious(next.measurement?.measured.timestamp ?? null);
      setSaved(true);
    } catch (e) {
      report(e);
    } finally {
      setBusy(false);
    }
  };
  const deleteTouch = async (index: number) => {
    setBusy(true);
    setAuto(false);
    setSaved(false);
    try {
      setStatus(await api.delete(index));
      setStep(0);
    } catch (e) {
      report(e);
    } finally {
      setBusy(false);
    }
  };
  const fit: TankFit | null = status?.fit ?? null;
  const fitReady = !!fit?.offsets_mm;
  const ready = active && !!status?.still && !status.stale;
  const touchCounts =
    status?.touches.reduce<Record<string, number>>(
      (counts, touch) => ({
        ...counts,
        [touch.label]: (counts[touch.label] ?? 0) + 1,
      }),
      {},
    ) ?? {};

  return (
    <Dialog
      open={open}
      onOpenChange={(value) => {
        if (!value) void close();
      }}
    >
      <DialogContent
        className="max-w-5xl max-h-[94vh] overflow-y-auto"
        onPointerDownOutside={(e) => e.preventDefault()}
      >
        <DialogHeader>
          <DialogTitle>{t("tank.title", { robot: robotName })}</DialogTitle>
          <DialogDescription>{t("tank.instruction")}</DialogDescription>
        </DialogHeader>
        <div className="flex flex-wrap items-center gap-3">
          {!active ? (
            <Button disabled={busy} onClick={() => void start()}>
              {t("tank.start")}
            </Button>
          ) : (
            <Button
              variant="outline"
              disabled={busy}
              onClick={() => void stop()}
            >
              {t("tank.stop")}
            </Button>
          )}
          {previous && (
            <span className="text-xs text-muted-foreground">
              {t("tank.previous", { time: previous })}
            </span>
          )}
        </div>
        {status?.trace_path && (
          <p className="text-xs text-muted-foreground break-all">
            {t("tank.trace", {
              count: status.trace_samples,
              path: status.trace_path,
            })}
          </p>
        )}
        {(error || status?.error) && (
          <p role="alert" className="text-destructive">
            {error ?? status?.error}
          </p>
        )}
        <div className="grid gap-5 md:grid-cols-2">
          <div className="space-y-4">
            <section>
              <h3 className="font-semibold">{t("tank.guide")}</h3>
              <p className="text-xs text-muted-foreground my-2">
                {t("tank.spacing")}
              </p>
              <ol className="space-y-1">
                {STEP_KEYS.map((key, i) => {
                  const required = STEPS.slice(0, i + 1).filter(
                    (f) => f === STEPS[i],
                  ).length;
                  const done =
                    i === 7
                      ? status?.touches.some((x) =>
                          x.label.startsWith("check:"),
                        )
                      : (touchCounts[STEPS[i]] ?? 0) >= required;
                  return (
                    <li key={key}>
                      <button
                        className={`w-full text-left rounded px-2 py-1 text-sm ${step === i && !extraTarget ? "bg-primary/10 font-semibold" : ""}`}
                        onClick={() => {
                          setStep(i);
                          setExtraTarget(null);
                        }}
                        aria-current={
                          step === i && !extraTarget ? "step" : undefined
                        }
                      >
                        {done ? "✓" : `${i + 1}.`} {t(`tank.${key}`)}
                      </button>
                    </li>
                  );
                })}
              </ol>
              <label className="text-sm flex gap-2 mt-2">
                {t("tank.checkFace")}
                <select
                  value={checkFace}
                  onChange={(e) => setCheckFace(e.target.value as TankFace)}
                  className="bg-background border rounded"
                >
                  {WALLS.map((f) => (
                    <option key={f} value={f}>
                      {face(f)}
                    </option>
                  ))}
                </select>
              </label>
            </section>
            <section
              className="rounded border p-3 space-y-2 text-sm"
              aria-live="polite"
            >
              <p
                className={
                  status?.stale
                    ? "text-destructive"
                    : ready
                      ? "text-green-600"
                      : "text-muted-foreground"
                }
              >
                {status?.stale
                  ? t("tank.stale")
                  : ready
                    ? t("tank.still")
                    : t("tank.moving")}
              </p>
              <p className="font-mono">
                {t("tank.probe", {
                  xyz: status?.probe_mm?.map(fmt).join(", ") ?? "—",
                })}
              </p>
              <p>
                {t("tank.opening", { value: fmt(status?.gripper_opening_mm) })}
              </p>
              {(status?.gripper_opening_mm ?? 0) > 5 && (
                <p className="text-amber-600">{t("tank.openWarning")}</p>
              )}
              {target ? (
                <Button
                  className="w-full"
                  disabled={
                    !ready ||
                    busy ||
                    (!target.startsWith("envelope:") &&
                      (status?.gripper_opening_mm ?? 100) > 5)
                  }
                  onClick={() => void capture()}
                >
                  {t("tank.capture", { label: label(target) })}
                </Button>
              ) : (
                <p>{t("tank.done")}</p>
              )}
              <label className="flex gap-2">
                <input
                  type="checkbox"
                  checked={auto}
                  onChange={(e) => setAuto(e.target.checked)}
                />
                {t("tank.auto")}
              </label>
              {!moved.current && (
                <p className="text-xs text-muted-foreground">
                  {t("tank.moveAgain")}
                </p>
              )}
            </section>
            <details className="rounded border p-3 text-sm">
              <summary className="cursor-pointer font-medium">
                {t("tank.tower")}
              </summary>
              <p className="my-2 text-muted-foreground">
                {t("tank.towerHint")}
              </p>
              <div className="flex flex-wrap gap-2">
                {TOWER_LABELS.map((l) => (
                  <Button
                    key={l}
                    size="sm"
                    variant={extraTarget === l ? "default" : "outline"}
                    onClick={() => setExtraTarget(l)}
                  >
                    {touchCounts[l] ? "✓ " : ""}
                    {t(`tank.${l}`)}
                  </Button>
                ))}
              </div>
            </details>
            <details className="rounded border p-3 text-sm">
              <summary className="cursor-pointer font-medium">
                {t("tank.envelope")}
              </summary>
              <p className="my-2 text-muted-foreground">
                {t("tank.envelopeHint")}
              </p>
              <label className="flex justify-between mb-2">
                {t("tank.link")}
                <select
                  className="bg-background border rounded"
                  value={link}
                  onChange={(e) => setLink(e.target.value)}
                >
                  {Object.keys(status?.paddings ?? {}).map((n) => (
                    <option key={n}>{n}</option>
                  ))}
                  {!status?.paddings.yaw_link && <option>yaw_link</option>}
                </select>
              </label>
              <label className="flex justify-between mb-2">
                {t("tank.face")}
                <select
                  className="bg-background border rounded"
                  value={envelopeFace}
                  onChange={(e) => setEnvelopeFace(e.target.value as TankFace)}
                >
                  {[...WALLS, "floor", "rim"].map((f) => (
                    <option key={f} value={f}>
                      {face(f)}
                    </option>
                  ))}
                </select>
              </label>
              <Button
                size="sm"
                variant="outline"
                disabled={!fit?.valid}
                onClick={() =>
                  setExtraTarget(`envelope:${link}:${envelopeFace}`)
                }
              >
                {t("tank.useEnvelope")}
              </Button>
            </details>
            {extraTarget && (
              <div className="text-sm">
                <p>{t("tank.optionalTarget", { label: label(extraTarget) })}</p>
                <Button variant="link" onClick={() => setExtraTarget(null)}>
                  {t("tank.useGuide")}
                </Button>
              </div>
            )}
          </div>
          <div className="space-y-4">
            <TankDrawing status={status} />
            <section className="rounded border p-3 text-sm space-y-2">
              <h3 className="font-semibold">{t("tank.fit")}</h3>
              {fitReady && (
                <>
                  <p>{t("tank.width", { value: fmt(fit?.width_mm) })}</p>
                  <p>{t("tank.yaw", { value: fmt(fit?.yaw_deg) })}</p>
                  <p>
                    {t("tank.rms", {
                      rms: fmt(fit?.rms_mm),
                      max: fmt(fit?.max_mm),
                    })}
                  </p>
                  <div className="grid grid-cols-2 gap-1">
                    {Object.entries(fit?.j1_distances_mm ?? {}).map(
                      ([f, d]) => (
                        <p key={f}>
                          {t("tank.j1Distance", {
                            face: face(f),
                            value: fmt(d),
                          })}
                          {f === "far" ? ` · ${t("tank.estimated")}` : ""}
                        </p>
                      ),
                    )}
                  </div>
                </>
              )}
              <p className={fit?.valid ? "text-green-600" : "text-amber-600"}>
                {fit?.valid ? t("tank.pass") : t("tank.fail")}
              </p>
              {!!fit?.failures.length && (
                <ul className="list-disc pl-5">
                  {fit.failures.map((f) => (
                    <li key={f}>{f}</li>
                  ))}
                </ul>
              )}
              {fit?.change && (
                <p>
                  {t("tank.change", {
                    distance: fmt(fit.change.translation_mm),
                    face: face(fit.change.toward),
                    rotation: fmt(fit.change.rotation_deg),
                  })}
                </p>
              )}
              <Button
                disabled={!active || !fit?.valid || busy}
                onClick={() => void save()}
              >
                {t("tank.save")}
              </Button>
              {saved && <p role="status">{t("tank.saved")}</p>}
            </section>
            {!!status?.touches.length && (
              <details className="rounded border p-3 text-sm" open>
                <summary>{t("tank.touches")}</summary>
                <ol className="max-h-48 overflow-auto mt-2 space-y-1">
                  {status.touches.map((touch, i) => (
                    <li
                      key={`${i}-${touch.timestamp}`}
                      className="flex justify-between gap-2 items-center"
                    >
                      <span>
                        {label(touch.label)} ·{" "}
                        {touch.point_mm.map(fmt).join(", ")}
                        {fit?.residuals?.find((r) => r.index === i) && (
                          <small className="block">
                            {t("tank.residual", {
                              value: fmt(
                                fit.residuals.find((r) => r.index === i)
                                  ?.residual_mm,
                              ),
                            })}
                          </small>
                        )}
                      </span>
                      <Button
                        aria-label={t("tank.delete", { index: i + 1 })}
                        variant="ghost"
                        size="sm"
                        disabled={!active || busy}
                        onClick={() => void deleteTouch(i)}
                      >
                        ×
                      </Button>
                    </li>
                  ))}
                </ol>
              </details>
            )}
          </div>
        </div>
        {status?.clearances && (
          <section className="text-sm space-y-2">
            <h3 className="font-semibold">{t("tank.clearances")}</h3>
            <p className="text-xs text-muted-foreground">
              {t("tank.clearanceHint")}
            </p>
            <div className="grid gap-2 md:grid-cols-2">
              {Object.entries(status.clearances.links).map(([name, values]) => (
                <div key={name} className="rounded border p-2">
                  <strong>{name}</strong>
                  <span className="block text-xs text-muted-foreground">
                    {t("tank.padding", {
                      value: fmt(values.padding_mm),
                      source:
                        values.padding_status === "default"
                          ? t("tank.defaultPadding")
                          : t("tank.measuredPadding"),
                    })}
                  </span>
                  <div className="flex flex-wrap gap-x-3">
                    {Object.entries(values.distances_mm).map(([f, d]) => (
                      <span
                        key={f}
                        className={
                          f !== "floor" &&
                          d <
                            (status.clearances?.margins_mm[f as TankFace] ?? 30)
                            ? "text-destructive"
                            : ""
                        }
                      >
                        {face(f)}: {fmt(d)}
                      </span>
                    ))}
                  </div>
                </div>
              ))}
            </div>
          </section>
        )}
        <p className="sticky bottom-0 z-10 border-t bg-background py-3 font-medium text-sm">
          {t("tank.measurementOnly")}
        </p>
      </DialogContent>
    </Dialog>
  );
}

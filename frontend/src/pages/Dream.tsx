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
import { useEffect, useRef, useState } from "react";
import { Link, useSearchParams } from "react-router-dom";
import { useTranslation } from "react-i18next";
import { ArrowLeft, ArrowLeftRight, ArrowUpRight, Camera, Check, Loader2, Pause, Play, Settings2, Sparkles } from "lucide-react";
import BrandMark from "@/components/BrandMark";
import LanguageSwitcher from "@/components/LanguageSwitcher";
import { Button } from "@/components/ui/button";
import { Dialog, DialogContent, DialogDescription, DialogHeader, DialogTitle } from "@/components/ui/dialog";
import { AlertDialog, AlertDialogAction, AlertDialogCancel, AlertDialogContent, AlertDialogDescription, AlertDialogFooter, AlertDialogHeader, AlertDialogTitle } from "@/components/ui/alert-dialog";
import { useApi } from "@/contexts/ApiContext";
import { useArms } from "@/hooks/useArms";
import { useRobots } from "@/hooks/useRobots";
import { useSessionHeartbeat } from "@/hooks/useSessionHeartbeat";
import { useUnloadWarning } from "@/hooks/useUnloadWarning";
import { apiRequest } from "@/lib/apiClient";
import { getCurrentSession, startSession, stopSession, type SessionInfo } from "@/lib/sessionApi";
import { tabOwnerId } from "@/lib/sessionOwner";
import { DREAM_DEFAULTS, DREAM_EXAMPLES, dreamCropBand, dreamApprove, dreamPlan, dreamPreview, type DreamPlan, type DreamPreview, type DreamReplay, type DreamSettings, type DreamSlot, type DreamStatus } from "@/lib/dreamApi";
import { cn } from "@/lib/utils";

const SELECT = "h-10 w-full rounded-xl border border-border bg-background px-3 text-sm outline-none focus:ring-2 focus:ring-ring";
const EMPTY: DreamStatus = { active: false, checking_inputs: false, run: null, robot: null, status: null, error: null, stale: false, button_alive: false };
const SLOTS: DreamSlot[] = ["wrist_l", "overhead", "wrist_r"];

export default function Dream() {
  const { t } = useTranslation();
  const { baseUrl, fetchWithHeaders } = useApi();
  const { byId } = useArms();
  const { records, selectedRecord } = useRobots();
  const [query] = useSearchParams();
  const eligible = Object.values(records).filter((r) => byId(r.arm_type)?.capabilities.supports_dream && r.mode === "single" && r.arms !== "leader");
  const [name, setName] = useState(query.get("robot") ?? selectedRecord?.name ?? "");
  const robot = eligible.find((r) => r.name === name) ?? eligible[0];
  const [settings, setSettings] = useState<DreamSettings>(DREAM_DEFAULTS);
  const [settingsOpen, setSettingsOpen] = useState(false);
  const [camerasOpen, setCamerasOpen] = useState(false);
  const [centre, setCentre] = useState<DreamSlot>("overhead");
  const [status, setStatus] = useState<DreamStatus>(EMPTY);
  const [session, setSession] = useState<SessionInfo | null>(null);
  const [showLastRun, setShowLastRun] = useState(true);
  const [preview, setPreview] = useState<DreamPreview | null>(null);
  const [plan, setPlan] = useState<DreamPlan | null>(null);
  const [pending, setPending] = useState<"check" | "plan" | "start" | "approve" | "stop" | null>(null);
  const [error, setError] = useState<string | null>(null);
  const [hint, setHint] = useState(0);
  const [sizes, setSizes] = useState<Record<string, { width: number; height: number }>>({});
  const [demo, setDemo] = useState(query.get("demo") === "1");
  const [replay, setReplay] = useState<DreamReplay | null>(null);
  const [playing, setPlaying] = useState(true);
  const [seek, setSeek] = useState(0);
  const [owner] = useState(tabOwnerId);
  const lastRobot = useRef(robot?.name);
  const live = status.active || session?.kind === "dream";
  const locked = !!pending || live || !!plan;
  const working = live || (demo && replay != null && replay.status.state !== "released");
  useSessionHeartbeat(session?.kind === "dream" ? session.id : null, owner, session?.kind === "dream" && session.owner === owner);
  useUnloadWarning(live);

  useEffect(() => {
    if (live) { setShowLastRun(true); setPreview(null); }
  }, [live]);

  useEffect(() => {
    const id = setInterval(() => setHint((n) => (n + 1) % DREAM_EXAMPLES.length), 5000);
    return () => clearInterval(id);
  }, []);

  useEffect(() => {
    let gone = false;
    const abort = new AbortController();
    let timer: ReturnType<typeof setTimeout>;
    const poll = async () => {
      try {
        const [next, current] = await Promise.all([
          apiRequest<DreamStatus>(baseUrl, fetchWithHeaders, "/api/v1/dream/status", { signal: abort.signal }),
          getCurrentSession(baseUrl, fetchWithHeaders, abort.signal),
        ]);
        if (!gone) {
          setStatus(next); setSession(current.session);
          if (current.session?.kind === "dream" && current.session.robot) setName(current.session.robot);
        }
      } catch (e) {
        if (!gone) setError(e instanceof Error ? e.message : t("dream.failed"));
      } finally {
        if (!gone) timer = setTimeout(poll, 600);
      }
    };
    void poll();
    return () => { gone = true; abort.abort(); clearTimeout(timer); };
  }, [baseUrl, fetchWithHeaders, t]);

  useEffect(() => {
    if (!demo) return;
    let gone = false;
    const abort = new AbortController();
    const origin = performance.now();
    let timer: ReturnType<typeof setTimeout>;
    const poll = async () => {
      const at = seek + (playing ? (performance.now() - origin) / 1000 : 0);
      try {
        const next = await apiRequest<DreamReplay>(baseUrl, fetchWithHeaders, `/api/v1/dream/replay?at_s=${at.toFixed(3)}`, { signal: abort.signal });
        if (!gone) {
          setReplay(next);
          if (next.elapsed_s >= next.duration_s && playing) { setSeek(0); setPlaying(false); }
        }
      } catch (e) {
        if (!gone) { setError(e instanceof Error ? e.message : t("dream.failed")); setPlaying(false); }
      } finally { if (!gone && playing) timer = setTimeout(poll, 200); }
    };
    void poll();
    return () => { gone = true; abort.abort(); clearTimeout(timer); };
  }, [demo, seek, playing, baseUrl, fetchWithHeaders, t]);

  useEffect(() => {
    if (lastRobot.current !== robot?.name) { setPreview(null); setPlan(null); setShowLastRun(false); lastRobot.current = robot?.name; }
  }, [robot?.name]);

  const change = (values: Partial<DreamSettings>) => {
    setSettings((old) => ({ ...old, ...values })); setShowLastRun(false); setPreview(null); setPlan(null); setError(null);
  };
  const act = async (kind: NonNullable<typeof pending>, fn: () => Promise<void>) => {
    setPending(kind); setError(null);
    try { await fn(); } catch (e) { setError(e instanceof Error ? e.message : t("dream.failed")); }
    finally { setPending(null); }
  };
  const inputs = { ...settings, prompt: settings.prompt || DREAM_EXAMPLES[0] };
  const view = demo ? replay?.status : status.status;
  const run = demo ? replay?.run : live ? status.run : preview?.run ??
    (showLastRun && status.robot === robot?.name && status.status?.mode === "live" ? status.run : null);
  const snapshot = !!run && (demo || live || !!preview || status.status?.mode === "live");
  const labels: Record<DreamSlot, string> = { overhead: t("dream.front"), wrist_l: t("dream.left"), wrist_r: t("dream.right") };
  const sourceFor = (slot: DreamSlot): string | null => {
    if (snapshot && view?.run === run) {
      const source = view.slots[slot];
      return source.startsWith("copy:") ? view.slots[source.slice(5) as DreamSlot] : source;
    }
    if (slot === "overhead") return settings.overhead;
    if (slot === "wrist_l") return settings.wrist_l;
    return settings.wrist_r === "copy" ? settings.wrist_l : settings.wrist_r.replace(/^camera:/, "");
  };
  const ordered: DreamSlot[] = [SLOTS.find((s) => s !== centre)!, centre, SLOTS.filter((s) => s !== centre)[1]];
  const alarm = error || (!demo && status.error) || (!demo && status.stale && t("dream.stale"));
  const reason = view?.last_reason;
  const badReason = !!reason && (view?.state === "refused" || /trip|refus|error|limp|held/i.test(reason));
  const cameraMapping = (
    <div className="space-y-5">
      <p className="text-sm text-muted-foreground">{t("dream.mappingNote")}</p>
      {(["overhead", "wrist_l", "wrist_r"] as DreamSlot[]).map((slot) => {
        const value = slot === "overhead" ? settings.overhead ?? "" : settings[slot];
        return <label key={slot} className="block space-y-2 text-sm font-medium">
          <span>{labels[slot]} <span className="font-mono text-xs text-muted-foreground">{slot}</span></span>
          <select aria-label={`${labels[slot]} (${slot})`} className={SELECT} disabled={locked || demo} value={value}
            onChange={(e) => change({ [slot]: e.target.value || null })}>
            {slot === "overhead" && <option value="">{t("dream.chooseCamera")}</option>}
            {slot === "wrist_l" && !robot?.cameras.some((c) => c.name === "wrist") && <option value="wrist">wrist</option>}
            <option value="black">{t("dream.black")}</option>
            {slot === "wrist_r" && <option value="copy">{t("dream.copyLeft")}</option>}
            {robot?.cameras.map((c) => <option key={c.id} value={slot === "wrist_r" ? `camera:${c.name}` : c.name}>{c.name}</option>)}
          </select>
        </label>;
      })}
    </div>
  );

  return <div className="min-h-screen bg-background text-foreground">
    <header className="flex flex-wrap items-center justify-between gap-4 border-b border-border/60 px-5 py-5 sm:px-9">
      <Link to="/" aria-label={t("dream.back")}><BrandMark /></Link>
      <div className="flex items-center gap-2">
        <span className="mr-3 hidden whitespace-nowrap text-xs text-muted-foreground md:inline">{t("dream.subtitle")}</span>
        <select aria-label={t("dream.robot")} className={cn(SELECT, "max-w-56 rounded-full")} value={robot?.name ?? ""} disabled={locked || demo}
          onChange={(e) => { setName(e.target.value); setPreview(null); setError(null); }}>
          {!robot && <option value="">{t("dream.chooseRobot")}</option>}
          {eligible.map((r) => <option key={r.name} value={r.name}>{r.name}</option>)}
        </select>
        <Button size="icon" variant="ghost" aria-label={t("dream.settings")} onClick={() => setSettingsOpen(true)}><Settings2 className="h-4 w-4" /></Button>
        <Button size="icon" variant="ghost" aria-label={t("dream.cameras")} onClick={() => setCamerasOpen(true)}><Camera className="h-4 w-4" /></Button>
      </div>
    </header>

    <main className="mx-auto flex max-w-[1440px] flex-col items-center gap-9 px-5 pb-14 pt-8 sm:px-9 lg:pt-14">
      <div className="flex w-full items-center justify-between text-xs text-muted-foreground">
        <span className="flex items-center gap-2"><span className={cn("h-1.5 w-1.5 rounded-full", demo ? "bg-amber-500" : live ? "bg-emerald-500 animate-pulse" : "bg-muted-foreground/50")} />{demo ? t("dream.demoLabel") : t("dream.title")}</span>
        <Button variant="ghost" size="sm" className="h-7 gap-2 rounded-full text-xs" disabled={live || !!pending} onClick={() => { setDemo(!demo); setError(null); setPlan(null); setSeek(0); setPlaying(true); }}>
          {demo ? t("dream.exitDemo") : t("dream.demo")}<ArrowUpRight className="h-3 w-3" />
        </Button>
      </div>
      <section aria-label={t("dream.cameras")} className="grid w-full grid-cols-2 items-center gap-4 md:grid-cols-[1fr_2.25fr_1fr] md:gap-5">
        {ordered.map((slot, i) => {
          const source = sourceFor(slot);
          const cam = robot?.cameras.find((c) => c.name === source);
          const black = source === "black";
          const showRaw = !demo && !snapshot && !status.checking_inputs && pending !== "check" && !live && !!cam;
          const hasSnapshot = snapshot && view?.run === run && view.images_written != null;
          const size = sizes[slot] ?? { width: cam?.width ?? 640, height: cam?.height ?? 480 };
          const src = hasSnapshot ? `${baseUrl}/api/v1/dream/view/${encodeURIComponent(run!)}/${slot}?v=${view?.images_written ?? 0}` :
            showRaw ? `${baseUrl}/api/v1/camera-preview/${cam.camera_index ?? 0}?rotation=${cam.rotation ?? 0}&unique_id=${encodeURIComponent(cam.unique_id ?? "")}` : null;
          return <figure key={slot} className={cn("min-w-0 space-y-3", i === 1 ? "order-first col-span-2 md:order-none md:col-span-1" : "")} data-slot={slot}>
            <div className={cn("relative overflow-hidden rounded-2xl border border-border/40 bg-black shadow-sm", i === 1 ? "aspect-[320/176]" : "aspect-[320/176] md:opacity-90")}>
              {src && <img key={src.split("?v=")[0]} src={src} alt={`${labels[slot]} · ${source ?? slot}`} className={cn("h-full w-full object-fill", showRaw && "object-contain")}
                style={showRaw ? { objectFit: "contain" } : undefined}
                onError={() => setError(t("dream.imageUnavailable"))}
                onLoad={(e) => { if (showRaw && e.currentTarget.naturalWidth && (size.width !== e.currentTarget.naturalWidth || size.height !== e.currentTarget.naturalHeight)) setSizes((old) => ({ ...old, [slot]: { width: e.currentTarget.naturalWidth, height: e.currentTarget.naturalHeight } })); }} />}
              {showRaw && <div aria-hidden="true" className="pointer-events-none absolute border border-white/80" style={{ ...dreamCropBand(size.width, size.height), boxShadow: "0 0 0 999px rgba(0,0,0,0.45)" }} />}
              {!src && !black && <div className="absolute inset-0 flex items-center justify-center text-xs text-white/40"><Camera className="mr-2 h-4 w-4" />{t("dream.noCamera")}</div>}
              <span className="absolute left-3 top-3 rounded-full bg-black/50 px-3 py-1 text-[11px] font-medium text-white backdrop-blur-md">{labels[slot]}</span>
              {black && <span className="absolute bottom-3 left-3 rounded-full border border-white/20 px-2 py-0.5 text-[10px] text-white/70">{t("dream.black")}</span>}
              {i !== 1 && <Button size="icon" variant="ghost" className="absolute bottom-2 right-2 h-8 w-8 rounded-full bg-black/40 text-white hover:bg-black/60 hover:text-white"
                aria-label={t("dream.swap", { camera: labels[slot] })} onClick={() => setCentre(slot)}><ArrowLeftRight className="h-3.5 w-3.5" /></Button>}
            </div>
            <figcaption className="px-1 text-[11px] text-muted-foreground">{snapshot ? demo ? t("dream.replayCrop") : t("dream.exact") : t("dream.liveCrop")}</figcaption>
          </figure>;
        })}
      </section>

      {demo && <div className="w-full max-w-2xl space-y-3">
        <div className="flex items-center gap-4"><Button size="icon" variant="outline" className="shrink-0 rounded-full" aria-label={playing ? t("dream.pauseReplay") : t("dream.play")}
          onClick={() => { setSeek(replay?.elapsed_s ?? 0); setPlaying(!playing); }}>{playing ? <Pause className="h-4 w-4" /> : <Play className="h-4 w-4" />}</Button>
          <input aria-label={t("dream.timeline")} className="w-full accent-foreground" type="range" min={0} max={replay?.duration_s ?? 59} step={0.2} value={playing ? replay?.elapsed_s ?? seek : seek}
            onChange={(e) => { setSeek(Number(e.target.value)); setPlaying(false); }} />
          <span className="whitespace-nowrap font-mono text-xs text-muted-foreground">{(replay?.elapsed_s ?? 0).toFixed(1)} / {(replay?.duration_s ?? 59).toFixed(1)} s</span>
        </div><p className="text-xs text-muted-foreground">{t("dream.demoNote")}</p>
      </div>}

      <section className="w-full max-w-2xl space-y-5" aria-labelledby="dream-ask">
        <h1 id="dream-ask" className="text-center text-3xl font-medium tracking-tight sm:text-[2.5rem]">{t("dream.what")}</h1>
        {working ? <div className="flex flex-wrap items-center justify-between gap-4 rounded-2xl border p-5">
          <p className="max-w-md text-sm">{t("dream.working", { prompt: (demo ? replay?.status.prompt : status.status?.prompt) ?? inputs.prompt })}</p>
          <Button variant="outline" className="rounded-full" disabled={demo || pending === "stop" || session?.kind !== "dream"} onClick={() => void act("stop", async () => {
            if (session?.kind === "dream") await stopSession(baseUrl, fetchWithHeaders, session.id);
          })}>{pending === "stop" || session?.phase === "stopping" ? t("dream.stopping") : t("dream.rest")}</Button>
        </div> : <form className="overflow-hidden rounded-3xl border border-border bg-card p-2 shadow-sm" onSubmit={(e) => { e.preventDefault(); void act("plan", async () => {
          if (robot) setPlan(await dreamPlan(baseUrl, fetchWithHeaders, robot.name, inputs));
        }); }}>
          <textarea aria-label={t("dream.task")} rows={2} className="w-full resize-none bg-transparent px-4 py-3 text-base outline-none placeholder:text-muted-foreground/55"
            placeholder={DREAM_EXAMPLES[hint]} value={demo ? replay?.status.prompt ?? "" : settings.prompt} disabled={locked || demo} onChange={(e) => change({ prompt: e.target.value })} />
          <div className="flex items-center justify-between gap-3 px-3 pb-2">
            <select className="bg-transparent text-xs text-muted-foreground outline-none" aria-label={t("dream.model")} value="dreamlabs/dream-1-beta" disabled><option value="dreamlabs/dream-1-beta">{t("dream.beta")}</option></select>
            <Button type="submit" className="gap-2 rounded-full px-6" disabled={!robot || locked || demo || !preview?.inputs_checked}>
              {pending === "plan" ? <Loader2 className="h-3.5 w-3.5 animate-spin" /> : <Sparkles className="h-3.5 w-3.5" />}{t("dream.run")}
            </Button>
          </div>
        </form>}
        <div className="flex flex-wrap items-center justify-center gap-3 text-xs text-muted-foreground">
          {!live && !demo && <Button variant="outline" size="sm" className="gap-2 rounded-full" disabled={!robot || locked} onClick={() => void act("check", async () => {
            setPreview(null); setShowLastRun(false);
            if (robot) setPreview(await dreamPreview(baseUrl, fetchWithHeaders, robot.name, inputs));
          })}>{pending === "check" ? <Loader2 className="h-3.5 w-3.5 animate-spin" /> : <Camera className="h-3.5 w-3.5" />}{t("dream.check")}</Button>}
          <span>{t("dream.betaOnly")}</span>
        </div>
        {preview && !demo && !live && <div className="space-y-3 rounded-2xl border border-border bg-muted/30 p-5 text-sm">
          <p>{t("dream.review")}</p><div className="flex flex-wrap items-center justify-between gap-3">
            <code className="text-xs">{t("dream.inputs", { id: preview.inputs_id })}</code>
            <Button size="sm" className="gap-2 rounded-full" variant={preview.inputs_checked ? "outline" : "default"} disabled={locked || preview.inputs_checked} onClick={() => void act("approve", async () => {
              await dreamApprove(baseUrl, fetchWithHeaders, preview); setPreview({ ...preview, inputs_checked: true });
            })}><Check className="h-3.5 w-3.5" />{preview.inputs_checked ? t("dream.approved") : t("dream.approve")}</Button>
          </div>
        </div>}
        {alarm && <p role="alert" className="whitespace-pre-wrap break-words rounded-xl border border-destructive/40 bg-destructive/10 p-4 text-sm text-destructive">{alarm}</p>}
        {badReason && <p role="alert" className="rounded-xl bg-destructive/10 p-4 text-sm text-destructive">{reason}</p>}
        <div role="status" className="space-y-1 text-center text-xs text-muted-foreground">
          <p>{pending === "plan" ? t("dream.planning") : pending === "check" ? t("dream.checking") : view ? `${view.phase}${view.call ? ` · ${t("dream.call", { call: view.call, cap: view.max_calls })}` : ""}` : t("dream.idle")}</p>
          {reason && !badReason && <p>{reason}</p>}
          {view && <p>{t("dream.peak", { torque: view.peak_torque_nm.toFixed(2), temperature: view.peak_temp_c.toFixed(1) })}</p>}
          {live && <p>{status.button_alive && t("dream.button")}{" · "}{t("dream.buttonOnly")}</p>}
          {!robot && !demo && <p>{t("dream.noRobot")}</p>}
        </div>
      </section>

      {view?.measured && <details className="w-full max-w-2xl rounded-2xl border border-border/60 px-5 py-4 text-sm">
        <summary className="cursor-pointer text-muted-foreground">{t("dream.telemetry")}</summary>
        <p className="mt-4 text-xs text-muted-foreground">{t("dream.no3d")}</p>
        <table className="mt-4 w-full text-left text-xs"><thead className="text-muted-foreground"><tr><th className="py-2 font-normal">{t("dream.joint")}</th><th className="font-normal">{t("dream.measured")}</th><th className="font-normal">{t("dream.commanded")}</th></tr></thead><tbody>
          {Object.entries(view.measured).map(([joint, value]) => <tr key={joint} className="border-t border-border/40 font-mono"><td className="py-2">{joint}</td><td>{value.toFixed(2)}</td><td>{view.commanded?.[joint]?.toFixed(2) ?? "—"}</td></tr>)}
        </tbody></table>
      </details>}
      {demo && replay && <details className="w-full max-w-2xl text-xs text-muted-foreground"><summary className="cursor-pointer">{t("dream.phases")}</summary><pre className="mt-3 whitespace-pre-wrap leading-6">{replay.console.join("\n")}</pre></details>}
      <footer className="flex w-full max-w-2xl items-center justify-between border-t border-border/50 pt-5 text-xs text-muted-foreground"><Link to="/" className="flex items-center gap-1"><ArrowLeft className="h-3 w-3" />{t("dream.back")}</Link><LanguageSwitcher /></footer>
    </main>

    <Dialog open={settingsOpen} onOpenChange={setSettingsOpen}><DialogContent className="max-h-[90vh] overflow-y-auto sm:max-w-md"><DialogHeader><DialogTitle>{t("dream.settings")}</DialogTitle><DialogDescription>{t("dream.betaOnly")}</DialogDescription></DialogHeader>
      <label className="space-y-2 text-sm"><span>{t("dream.maxCalls")}</span><input type="number" className={SELECT} min={1} max={20} step={1} value={settings.max_calls} disabled={locked || demo} onChange={(e) => change({ max_calls: Number(e.target.value) })} /></label>
      <label className="space-y-2 text-sm"><span>{t("dream.pause")}</span><input type="number" className={SELECT} min={0} max={600} step={1} value={settings.pause_s} disabled={locked || demo} onChange={(e) => change({ pause_s: Number(e.target.value) })} /></label>
      <h3 className="mt-2 text-sm font-medium">{t("dream.mapping")}</h3>{cameraMapping}
      <Button className="mt-3 rounded-full" onClick={() => setSettingsOpen(false)}>{t("dream.close")}</Button>
    </DialogContent></Dialog>
    <Dialog open={camerasOpen} onOpenChange={setCamerasOpen}><DialogContent className="left-auto right-0 top-0 h-full max-h-screen translate-x-0 translate-y-0 overflow-y-auto rounded-none sm:max-w-sm"><DialogHeader><DialogTitle>{t("dream.cameras")}</DialogTitle><DialogDescription>{t("dream.mapping")}</DialogDescription></DialogHeader>{cameraMapping}
      <p className="text-xs text-muted-foreground">{t("dream.review")}</p><Button variant="outline" className="rounded-full" onClick={() => setCamerasOpen(false)}>{t("dream.close")}</Button>
    </DialogContent></Dialog>
    <AlertDialog open={!!plan} onOpenChange={(open) => { if (!open && pending !== "start") setPlan(null); }}><AlertDialogContent className="max-h-[90vh] max-w-2xl overflow-y-auto"><AlertDialogHeader><AlertDialogTitle>{t("dream.confirm")}</AlertDialogTitle><AlertDialogDescription>{t("dream.confirmNote")}</AlertDialogDescription></AlertDialogHeader>
      <pre className="overflow-x-auto rounded-xl bg-muted p-4 text-xs leading-6">{plan?.movement}</pre>
      <div className="space-y-2 text-sm"><h3 className="font-medium">{t("dream.readiness")}</h3><ul className="list-disc space-y-1 pl-5 text-xs text-muted-foreground">{plan?.readiness.map((line) => <li key={line}>{line}</li>)}</ul><p className="text-xs text-muted-foreground">{t("dream.readinessNote")}</p></div>
      <AlertDialogFooter><AlertDialogCancel disabled={pending === "start"}>{t("dream.cancel")}</AlertDialogCancel><AlertDialogAction disabled={!!pending} onClick={(event) => { event.preventDefault(); void act("start", async () => {
        if (!plan || !robot) return;
        const started = await startSession(baseUrl, fetchWithHeaders, { kind: "dream", robot: robot.name, owner, options: { plan_id: plan.plan_id } });
        setSession(started.session); setPlan(null);
      }); }}>{t("dream.confirmRun")}</AlertDialogAction></AlertDialogFooter>
    </AlertDialogContent></AlertDialog>
  </div>;
}

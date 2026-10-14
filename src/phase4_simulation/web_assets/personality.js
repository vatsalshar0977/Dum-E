/* Phase 7 panel: play B.U.D.-E's gestures at 100 Hz and parse typed commands.
   The frames come from the server (which samples the same Python engine the robot runs),
   so what you see here is the actual command stream, not a browser-side re-implementation. */
(function () {
  const $ = (id) => document.getElementById(id);
  const state = { sequences: {}, playing: null, timer: null };

  function say(html) { $('p7out').innerHTML = html; }

  function api(url) { return fetch(url).then((r) => r.json()); }

  // ---------------------------------------------------------------- the player ---
  function play(name) {
    api('/api/personality/sequence?name=' + encodeURIComponent(name)).then((d) => {
      if (!d.ok) { say('could not load ' + name + ': ' + (d.error || 'error')); return; }
      if (!window.BUDE) { say('the viewer hook is missing: app.js did not expose BUDE'); return; }
      stop();
      const frames = d.frames, hz = d.sample_hz, t0 = performance.now();
      const out = $('p7play');
      out.innerHTML = `<b>${d.name}</b> ${d.duration_s.toFixed(2)} s, ${frames.length} frames at ` +
        `${hz} Hz, priority ${d.priority}, speed x${d.speed_scale}` +
        (d.loop ? ', looping until interrupted' : '') +
        `<br><span class="dim">peak joint speed ` +
        d.peak_speed_deg_s.slice(0, 6).map((v, i) => `J${i + 1} ${v.toFixed(1)}`).join(' &middot; ') +
        ` &deg;/s</span><br><span class="dim">` +
        `wheel command ${frames[Math.min(frames.length - 1, 20)].wheels[0].toFixed(0)} us ` +
        `(1500 = stop)</span>`;
      const step = () => {
        const elapsed = (performance.now() - t0) / 1000;
        let index = Math.floor(elapsed * hz);
        if (d.loop) { index = index % frames.length; }
        if (index >= frames.length) { stop(); out.innerHTML += '<br><b>done</b>'; return; }
        const f = frames[index];
        window.BUDE.setJoints(f.joints.slice(0, 5));
        window.BUDE.setGripper(1 - Math.min(1, Math.max(0, f.joints[5] / 60)));   // 0 deg = shut
        state.timer = requestAnimationFrame(step);
      };
      state.playing = d.name;
      state.timer = requestAnimationFrame(step);
    });
  }

  function stop() {
    if (state.timer) { cancelAnimationFrame(state.timer); state.timer = null; }
    state.playing = null;
  }

  // ---------------------------------------------------------------- commands ----
  function speak(text) {
    if (!text) { say('type something first, e.g. <i>hand me the screwdriver</i>'); return; }
    say('parsing &hellip;');
    api('/api/voice?text=' + encodeURIComponent(text)).then((d) => {
      if (!d.ok) { say('error: ' + (d.error || 'unknown')); return; }
      const i = d.intent;
      const cls = i.intent === 'HALT' ? 'bad' : i.intent === 'UNKNOWN' ? '' : 'ok';
      let html = `<b>${text}</b><br>` +
        `<span class="badge ${cls}">${i.intent}</span> ` +
        `<span class="dim">confidence ${i.confidence.toFixed(2)} &middot; ` +
        `${i.evidence ? i.evidence : 'no evidence'}</span><br>` +
        `&ldquo;${i.response}&rdquo;`;
      if (d.action_log && d.action_log.length) {
        html += '<br><span class="dim">the simulated robot did:</span><br>' +
          d.action_log.map((l) => '&bull; ' + l).join('<br>');
      }
      if (d.gestures && d.gestures.length) {
        html += '<br><span class="dim">gestures: ' + d.gestures.join(' &rarr; ') + '</span>';
      }
      say(html);
      if (i.intent === 'HALT') { stop(); if (window.BUDE) window.BUDE.halt(); return; }
      const first = (d.gestures || []).find((g) => state.sequences[g]);
      if (first) play(first);
    });
  }

  function boot() {
    api('/api/personality').then((d) => {
      if (!d.ok) { say('could not load the gesture library'); return; }
      d.sequences.forEach((s) => { state.sequences[s.name] = s; });
      $('p7btns').innerHTML = d.sequences.map((s) =>
        `<button data-seq="${s.name}" title="${s.description.replace(/"/g, '&quot;')}">` +
        `${s.name.replace(/_/g, ' ')}<br><span class="dim">${s.duration_s.toFixed(2)} s` +
        `${s.priority === 0 ? ' &middot; reflex' : ''}</span></button>`).join('');
      $('p7btns').querySelectorAll('[data-seq]').forEach((b) =>
        b.addEventListener('click', () => play(b.dataset.seq)));
      $('p7chips').innerHTML = d.known_phrases.map((p) =>
        `<button class="chip" data-say="${p}">${p}</button>`).join('');
      $('p7chips').querySelectorAll('[data-say]').forEach((b) =>
        b.addEventListener('click', () => { $('p7text').value = b.dataset.say; speak(b.dataset.say); }));
      say(`<span class="dim">${d.sequences.length} gestures at ${d.sample_hz} Hz minimum jerk. ` +
        `${d.note}</span>`);
    }).catch((e) => say('could not reach /api/personality: ' + e));
    $('p7go').addEventListener('click', () => speak($('p7text').value));
    $('p7text').addEventListener('keydown', (e) => { if (e.key === 'Enter') speak($('p7text').value); });
    $('p7stop').addEventListener('click', () => { stop(); if (window.BUDE) window.BUDE.halt(); });
  }

  if (document.readyState === 'loading') { document.addEventListener('DOMContentLoaded', boot); }
  else { boot(); }
})();

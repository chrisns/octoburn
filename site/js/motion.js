/* OctoBurn scroll motion: a pinned six-frame story, depth parallax, zooms and reveals.
   No library. Everything is off for prefers-reduced-motion, and the page reads fine without JS. */
(function () {
  "use strict";
  var reduce = window.matchMedia && window.matchMedia("(prefers-reduced-motion: reduce)").matches;
  document.documentElement.classList.add(reduce ? "motion-off" : "motion-on");
  if (reduce) return;

  var clamp = function (v, a, b) { return Math.max(a, Math.min(b, v)); };
  var ease = function (t) { return t * t * (3 - 2 * t); };

  /* Reveals: fade and rise once, staggered within a row. */
  var reveals = [].slice.call(document.querySelectorAll("[data-reveal]"));
  if ("IntersectionObserver" in window) {
    var io = new IntersectionObserver(function (entries) {
      entries.forEach(function (e) {
        if (!e.isIntersecting) return;
        var el = e.target, sibs = [].slice.call(el.parentNode.children).filter(function (c) { return c.hasAttribute("data-reveal"); });
        el.style.transitionDelay = (Math.max(0, sibs.indexOf(el)) % 3) * 90 + "ms";
        el.classList.add("is-in");
        io.unobserve(el);
      });
    }, { rootMargin: "0px 0px -12% 0px", threshold: 0.12 });
    reveals.forEach(function (el) { io.observe(el); });
  } else {
    reveals.forEach(function (el) { el.classList.add("is-in"); });
  }

  var speeds = [].slice.call(document.querySelectorAll("[data-speed]"));
  var zooms = [].slice.call(document.querySelectorAll("[data-zoom]"));
  var tilts = [].slice.call(document.querySelectorAll("[data-tilt]"));
  var inners = [].slice.call(document.querySelectorAll("[data-inner]"));
  var story = document.querySelector(".ob-story");
  var panels = story ? [].slice.call(story.querySelectorAll(".ob-story-panel")) : [];
  var dots = story ? [].slice.call(story.querySelectorAll(".ob-story-progress span")) : [];
  var ticking = false;

  function frame() {
    ticking = false;
    var vh = window.innerHeight, mid = vh / 2;

    /* Depth: an element drifts by its distance from the middle of the screen times its speed. */
    speeds.forEach(function (el) {
      var r = el.getBoundingClientRect();
      if (r.bottom < -vh || r.top > 2 * vh) return;
      var d = (r.top + r.height / 2) - mid;
      el.style.transform = "translate3d(0," + (d * parseFloat(el.getAttribute("data-speed"))).toFixed(1) + "px,0)";
    });

    /* Zoom: the film plate grows from 92 % to full width as it arrives. */
    zooms.forEach(function (el) {
      var r = el.getBoundingClientRect();
      var t = ease(clamp((vh - r.top) / (vh * 0.8), 0, 1));
      el.style.transform = "scale(" + (0.92 + 0.08 * t).toFixed(4) + ")";
    });

    /* Tilt: the probe turntable leans into view. */
    tilts.forEach(function (el) {
      var r = el.getBoundingClientRect();
      var t = clamp(((r.top + r.height / 2) - mid) / vh, -1, 1);
      var base = el.style.transform.replace(/ ?(perspective|rotateX|rotateY)\([^)]*\)/g, "");
      el.style.transform = base + " perspective(1200px) rotateX(" + (t * 8).toFixed(2) + "deg) rotateY(" + (-t * 6).toFixed(2) + "deg)";
    });

    /* Card thumbs: the picture slides inside its frame. */
    inners.forEach(function (el) {
      var r = el.parentNode.getBoundingClientRect();
      if (r.bottom < 0 || r.top > vh) return;
      var t = ((r.top + r.height / 2) - mid) / vh;
      el.style.transform = "scale(1.12) translate3d(0," + (t * -18).toFixed(1) + "px,0)";
    });

    /* Story: the stage is pinned while the section scrolls past; each sixth is one frame. */
    if (story) {
      var r = story.getBoundingClientRect();
      var span = story.offsetHeight - vh;
      var p = clamp(-r.top / span, 0, 0.9999);
      var n = panels.length, f = p * n, idx = Math.floor(f), local = f - idx;
      panels.forEach(function (el, i) {
        var on = i === idx;
        el.classList.toggle("is-active", on);
        var lp = i < idx ? 1 : i > idx ? 0 : local;
        el.style.setProperty("--lp", lp.toFixed(4));
      });
      dots.forEach(function (d, i) { d.classList.toggle("is-on", i <= idx); });
    }
  }

  function onScroll() {
    if (!ticking) { ticking = true; window.requestAnimationFrame(frame); }
  }
  window.addEventListener("scroll", onScroll, { passive: true });
  window.addEventListener("resize", onScroll);
  frame();
})();

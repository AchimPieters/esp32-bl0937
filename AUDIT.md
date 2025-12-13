# esp-bl0937 code audit

## Scope and approach
- Reviewed driver, calibration, NVS helpers, and policy helper in `src/` plus public headers in `include/`.
- Focused on correctness, edge-case handling, and opportunities to improve runtime efficiency or robustness.

## Status (follow-ups addressed)
- Events now support multiple listeners, and the policy helper registers without overwriting existing callbacks. Relay callbacks fire again when the overcurrent hold expires, removing the need for polling.
- CF/CF1 inputs accept configurable pull-up/down settings to avoid floating inputs on boards without external biasing.
- Sampling windows use `vTaskDelayUntil` to reduce drift while still measuring elapsed time precisely.
- Disabling the driver resets counters, filters, energy, and latches for predictable reuse.
- Calibration blobs gained a version byte; incompatible versions fail fast during load.

## Functional risks
1. **Single global event callback limits composability.** The driver stores only one callback, and the policy helper replaces any existing callback without preserving the previous registration, preventing applications from listening to events while also using the policy helper. Consider adding a small fan-out/observer list or allowing the policy helper to wrap rather than overwrite the existing handler. 【F:src/bl0937.c†L94-L99】【F:src/bl0937_policy.c†L69-L78】
2. **GPIO input configuration assumes external biasing.** CF and CF1 are configured as plain inputs with no pull-ups or pull-downs, which can leave lines floating on designs lacking external resistors and produce spurious pulses or overcounts. Exposing pull configuration in `bl0937_config_t` (or enabling safe defaults) would harden the driver across boards. 【F:src/bl0937.c†L145-L164】
3. **Relay policy only reacts on overcurrent rising edge.** The policy helper latches a hold window on `OVERCURRENT_ON` but never invokes the relay callback when the hold clears; callers must poll `bl0937_policy_allows_relay_on`. Publishing an explicit release notification (or invoking the relay callback when the hold expires) would reduce reliance on polling and avoid stale relay states. 【F:src/bl0937_policy.c†L38-L95】

## Optimization and robustness opportunities
1. **Sampling window timing accuracy.** `bl0937_sample` relies on `vTaskDelay` for the sampling window, which can overshoot under load; although the elapsed time is measured, longer windows can mask short spikes and slow overcurrent detection. Consider using `vTaskDelayUntil` to reduce drift or allowing a configurable high-resolution busy-wait window for critical measurements. 【F:src/bl0937.c†L291-L362】
2. **Energy and filter state lifecycle.** Disabling the meter or recreating handles leaves filter state and energy accumulator untouched until the next sample resets counters, which can surprise callers expecting a clean state after `bl0937_enable(h, false)` or before reuse. Adding explicit reset helpers (or auto-reset on disable/destroy) would make lifecycle behavior clearer. 【F:src/bl0937.c†L235-L399】
3. **NVS blob versioning.** Calibration blobs are validated with CRC but lack versioning, so schema changes could produce silent misreads. Storing a version byte and rejecting incompatible versions would make future upgrades safer. 【F:src/bl0937_nvs.c†L18-L47】

## Testing coverage gaps
- Unit tests cover only the filter utilities and notification helper; there are no tests for the driver’s pulse math, overcurrent latch behavior, NVS persistence, or policy helper workflows. Adding host-side or hardware-in-the-loop tests for these areas would raise confidence. 【F:test/test_bl0937_helpers.c†L1-L35】

## Aanbevolen aanpassingen (korte checklist)
- Voeg ondersteuning toe voor meerdere event-listeners of laat de policy-helper de bestaande callback doorgeven, zodat applicaties tegelijk eigen handlers kunnen registreren. (Functie)
- Breid `bl0937_config_t` uit met pull-up/pull-down opties voor CF/CF1 of documenteer veilige standaardwaarden, zodat instabiele signalen worden voorkomen. (Robuustheid)
- Laat de policy-helper een vrijgave-event sturen wanneer de overcurrent-hold vervalt, zodat relaisstatussen niet blijven hangen. (Functionaliteit)
- Optimaliseer de sample-timing met `vTaskDelayUntil` of een configureerbare high-resolution wachttijd om drift te beperken. (Performance)
- Voeg een reset-mogelijkheid toe voor energie- en filterstatus bij disable/destroy om hergebruik voorspelbaar te maken. (Lifecycle)
- Introduceer een versienummer in de NVS-kalibratieblob en weiger incompatibele versies. (Compatibiliteit)
- Breid tests uit met scenario’s voor puls-naar-vermogen-math, overcurrent-latch, NVS round-trip, en policy-helper workflows. (Testdekking)

# BL0937 (BL09x7) Energy Meter Component for ESP-IDF 5.x

This is a self-contained ESP-IDF component that reads **CF / CF1 / SEL** pulse outputs from a
BL09x7-style single-phase energy metering IC (BL0937 datasheet compatible) and exposes:

- Voltage (VRMS)
- Current (IRMS)
- Active power (W)
- Energy (Wh) from CF pulse accumulation
- Power factor estimate (P / (V * I))
- Optional **overcurrent protection** with relay cut-off and 30s cooldown

## Quick start

1) Add the component (from registry or as a git submodule)

2) Configure pins and scaling:
```text
idf.py menuconfig
  -> BL0937 (BL09x7) Energy Meter
```

3) Use in code:
```c
#include "bl0937.h"

void app_main(void)
{
    // Optional. If you don't call this, the component auto-inits on first getter call.
    bl0937_init_default();

    while (1) {
        printf("V=%.1fV I=%.3fA P=%.1fW E=%.3fWh PF=%.2f\n",
               bl0937_voltage(), bl0937_current(), bl0937_power(),
               bl0937_energy_wh(), bl0937_power_factor());
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
```

## Scaling & calibration

The component uses the BL0937 typical frequency formulas to compute input voltages,
then converts to mains voltage and current using:

- `divider_ratio = divider_num/divider_den`
- `shunt_uohm` for current

Use `*_CAL_*` options to correct systematic errors.
If enabled, calibration can be stored in NVS.

## Overcurrent protection

Software protection compares measured current against `OVERCURRENT_THRESHOLD_MA` with a debounce,
then disables the relay for `OVERCURRENT_COOLDOWN_S` seconds.

Optionally, CF can output a ~6.7kHz overcurrent pulse train (chip-level detection on some variants).
If enabled, the driver treats this as an immediate trip.

## License

Apache-2.0


## Apparent & reactive power

- Apparent power: `S = V * I` (VA)
- Reactive power magnitude: `Q = sqrt(S^2 - P^2)` (var)

> Let op: het teken (inductief/capacitief) kan je zonder fase-informatie niet betrouwbaar bepalen.
> Daarom geeft dit component **altijd een niet-negatieve Q**.

## Events (callback)

Je kunt optioneel een callback registreren:

```c
static void meter_evt(bl0937_event_t ev, void *ctx)
{
    (void)ctx;
    if (ev == BL0937_EVENT_OVERCURRENT_TRIP)   printf("TRIP!\n");
    if (ev == BL0937_EVENT_OVERCURRENT_RECOVER) printf("RECOVER\n");
    if (ev == BL0937_EVENT_HW_OVERCURRENT)      printf("HW OC\n");
}

void app_main(void)
{
    bl0937_init_default();
    bl0937_set_event_callback(meter_evt, NULL);
}
```


## Signal validity (detect missing pulses)

Het component markeert met flags of de metingen geldig zijn, gebaseerd op:
- time-out: `BL0937_SIGNAL_TIMEOUT_MS`
- optionele minimum pulsfreq: `BL0937_MIN_VALID_FREQ_HZ` (in mHz)

Gebruik:
- `bl0937_is_signal_ok()`
- of `bl0937_get_status(&st)` en lees `st.valid_voltage`, `st.valid_current`, `st.valid_power`.

## Energy persistence (NVS)

Als je `Persist energy counter (CF pulses) in NVS` aan zet:
- wordt de **CF pulse counter** periodiek opgeslagen (default elke 60s)
- na reboot gaat energie (Wh) door

Je kunt ook handmatig:
- `bl0937_energy_persist_now()`
- `bl0937_energy_restore()`


## Start/Stop control

Standaard start het component automatisch na `bl0937_init_default()`.

Wil je dat niet, zet dan in menuconfig:
- `Autostart measuring (start timer automatically)` = **OFF**

En gebruik dan:

```c
bl0937_init_default();
bl0937_start(); // starten wanneer jij wil
...
bl0937_stop();  // pauzeren
```

## Relay gedrag configureren

Als relay cutoff aan staat:
- `Relay initial state ON`
- `Relay defaults to ON after overcurrent recover`

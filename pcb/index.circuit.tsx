/**
 * Payload Rover — ESP32 Skid-Steer Motor Carrier Board
 * ------------------------------------------------------------------
 * Carrier PCB for the rover's rear electronics cabinet: an ESP32 DevKit
 * commands four socketed BLDC driver modules (JYQD-V7.3E class), one per
 * 10" hub motor. Each motor's cable lands on this board (3 phase + 5 hall)
 * and is passed through to its driver socket; the ESP32 only sees logic
 * (throttle / direction / brake / speed-pulse per wheel).
 *
 *  36V battery ──► J_BAT (inline MIDI fuse + E-stop contactor are OFF-BOARD,
 *        │          in the cabinet feed — this board never breaks motor power)
 *        ├──► 4x driver PWR sockets (V36/GND + phase out)
 *        └──► J_BUCK (Mini560-class 36V→5V module) ──► V5 ──► ESP32 VIN
 *
 *  per motor x ∈ {FL, FR, RL, RR}:
 *    ESP32 GPIO ──► VR_x (PWM throttle) / ZF_x (direction) / EL_x (brake)
 *    ESP32 GPIO ◄── M_x (speed pulse, input-only pins where possible)
 *    J_PH_x (3P screw terminal) ◄──► driver MA/MB/MC   (motor phases)
 *    J_HL_x (5p header)         ◄──► driver hall pins  (pass-through, module 5V)
 *
 *  GPIO map (30-pin NodeMCU):
 *    FL: VR=25 ZF=26 EL=27 M=34 | FR: VR=32 ZF=33 EL=14 M=35
 *    RL: VR=12 ZF=13 EL=23 M=36 | RR: VR=22 ZF=21 EL=19 M=39
 *    ESTOP sense=18, LED_STAT=4.  (GPIO12 is a boot-strap pin; it drives a
 *    VR throttle which idles LOW at boot — safe. 34/35/36/39 are input-only,
 *    used for the speed-pulse inputs.)
 *
 * FLOORPLAN (board 240 x 140 mm, origin = center, x∈[-120,120] y∈[-70,70])
 *  matches fusion/payload_car/control_pcb.py and the cabinet standoffs:
 *  - M3 mounting holes at (±113, ±63)
 *  - Top-left : J_BAT 36V in + bulk cap;  top-right: J_BUCK + J_ES + LEDs
 *  - Center   : ESP32 DevKit on two 15-pin female headers (USB faces UP)
 *  - Driver row: 4 sockets at x = -86/-29/+29/+86 (PWR @3.96mm, SIG @2.54mm)
 *  - Bottom   : per-motor 3P phase terminal (wire entry DOWN, toward the
 *               cabinet glands) + 5p hall header above it
 *
 * ⚠ FAB NOTES
 *  - Phase + V36 paths carry motor current (≈10 A continuous, more at stall):
 *    pour/widen these (the autorouted defaults are NOT enough), 2 oz copper.
 *  - E-STOP and the main fuse are intentionally NOT on this board: the red
 *    mushroom on the cabinet drives a contactor that cuts the 36 V feed
 *    upstream of J_BAT. J_ES here is only the latching-state SENSE input.
 *  - Driver modules vary (JYQD pinouts differ by revision) — verify the
 *    socket pinout against the purchased module before soldering headers.
 */

import { WJ500V_5_08_2P as Terminal2P } from "./imports/WJ500V_5_08_2P"
import { WJ128V_5_0_03P_14_00A as Terminal3P } from "./imports/WJ128V_5_0_03P_14_00A"

const BAT_PINS = { pin1: "V36", pin2: "GND" } as const
const ES_PINS = { pin1: "ES", pin2: "GND" } as const
const PH_PINS = { pin1: "MA", pin2: "MB", pin3: "MC" } as const

// driver socket rows (JYQD-V7.3E class module on two header strips)
const DRV_PWR_LABELS = ["V36", "GND", "MA", "MB", "MC"]
const DRV_SIG_LABELS = ["H5V", "HGND", "HA", "HB", "HC", "VR", "ZF", "EL", "M", "GND", "NC"]
const HALL_LABELS = ["H5V", "HGND", "HA", "HB", "HC"]

// full port paths because the GPIOs are split across the two DevKit headers
const MOTORS = [
  { id: "FL", x: -86, vr: "ESP_L.GPIO25", zf: "ESP_L.GPIO26", el: "ESP_L.GPIO27", m: "ESP_L.GPIO34", schY: 14 },
  { id: "FR", x: -29, vr: "ESP_L.GPIO32", zf: "ESP_L.GPIO33", el: "ESP_L.GPIO14", m: "ESP_L.GPIO35", schY: 4 },
  { id: "RL", x: 29, vr: "ESP_L.GPIO12", zf: "ESP_L.GPIO13", el: "ESP_R.GPIO23", m: "ESP_L.GPIO36", schY: -6 },
  { id: "RR", x: 86, vr: "ESP_R.GPIO22", zf: "ESP_R.GPIO21", el: "ESP_R.GPIO19", m: "ESP_L.GPIO39", schY: -16 },
] as const

export default () => (
  <board width="240mm" height="140mm">
    {/* M3 mounting holes — the cabinet standoff grid (see builder.py) */}
    <hole diameter="3.2mm" pcbX={-113} pcbY={63} />
    <hole diameter="3.2mm" pcbX={113} pcbY={63} />
    <hole diameter="3.2mm" pcbX={-113} pcbY={-63} />
    <hole diameter="3.2mm" pcbX={113} pcbY={-63} />

    {/* ============================================================ */}
    {/* POWER: 36V battery in (fused + contactor'd upstream) + buck   */}
    {/* ============================================================ */}
    <Terminal2P
      name="J_BAT"
      pinLabels={BAT_PINS}
      schX={-16}
      schY={10}
      pcbRotation={180}
      pcbX={-105}
      pcbY={58}
    />
    <capacitor
      name="C_BULK"
      capacitance="220uF"
      footprint="1206"
      schX={-13}
      schY={8}
      pcbX={-88}
      pcbY={58}
    />
    {/* Mini560-class buck MODULE on a 4-pin socket: 36V -> 5V/3A for the
       ESP32 VIN. The DevKit's own LDO makes 3V3. */}
    <pinheader
      name="J_BUCK"
      schWidth={0.58}
      pinCount={4}
      pitch="2.54mm"
      gender="female"
      showSilkscreenPinLabels
      pinLabels={["VIN", "GIN", "VOUT", "GOUT"]}
      schX={-13}
      schY={12}
      pcbX={62}
      pcbY={58}
    />
    <trace from="J_BAT.V36" to="net.V36" />
    <trace from="J_BAT.GND" to="net.GND" />
    <trace from="C_BULK.pin1" to="net.V36" />
    <trace from="C_BULK.pin2" to="net.GND" />
    <trace from="J_BUCK.VIN" to="net.V36" />
    <trace from="J_BUCK.GIN" to="net.GND" />
    <trace from="J_BUCK.VOUT" to="net.V5" />
    <trace from="J_BUCK.GOUT" to="net.GND" />

    {/* ============================================================ */}
    {/* ESP32 NodeMCU 30-pin DevKit on two 15-pin female headers      */}
    {/* (same socket pattern as the varuna controller, USB faces UP)  */}
    {/* ============================================================ */}
    <pinheader
      name="ESP_L"
      schWidth={0.77}
      pinCount={15}
      pitch="2.54mm"
      gender="female"
      pcbRotation={270}
      showSilkscreenPinLabels
      pinLabels={[
        "EN", "GPIO36", "GPIO39", "GPIO34", "GPIO35", "GPIO32", "GPIO33",
        "GPIO25", "GPIO26", "GPIO27", "GPIO14", "GPIO12", "GND_L", "GPIO13",
        "V5_L",
      ]}
      schX={0}
      schY={0}
      pcbX={-13}
      pcbY={30}
    />
    <pinheader
      name="ESP_R"
      schWidth={0.77}
      pinCount={15}
      pitch="2.54mm"
      gender="female"
      pcbRotation={270}
      showSilkscreenPinLabels
      pinLabels={[
        "GPIO23", "GPIO22", "GPIO1", "GPIO3", "GPIO21", "GPIO19", "GPIO18",
        "GPIO5", "GPIO17", "GPIO16", "GPIO4", "GPIO0", "GPIO2", "GPIO15",
        "V3V3",
      ]}
      schX={5}
      schY={0}
      pcbX={13}
      pcbY={30}
    />
    <silkscreentext text="ESP32 NodeMCU 30p" pcbX={0} pcbY={34} anchorAlignment="center" fontSize={1.4} />
    <silkscreentext text="USB ^ TOP" pcbX={0} pcbY={50} anchorAlignment="center" fontSize={1.2} />
    <trace from="ESP_L.V5_L" to="net.V5" />
    <trace from="ESP_L.GND_L" to="net.GND" />

    {/* ============================================================ */}
    {/* E-STOP latching-state sense (the mushroom + contactor that    */}
    {/* actually CUT power live in the cabinet, upstream of J_BAT)    */}
    {/* ============================================================ */}
    <Terminal2P
      name="J_ES"
      pinLabels={ES_PINS}
      schX={9}
      schY={10}
      pcbRotation={180}
      pcbX={105}
      pcbY={58}
    />
    <resistor name="R_ES" resistance="10k" footprint="0603" schX={6} schY={8} pcbX={95} pcbY={44} />
    <capacitor name="C_ES" capacitance="100nF" footprint="0603" schX={6} schY={6} pcbX={95} pcbY={40} />
    <trace from="J_ES.ES" to="ESP_R.GPIO18" schDisplayLabel="ESTOP_SENSE" />
    <trace from="J_ES.GND" to="net.GND" />
    <trace from="R_ES.pin1" to="ESP_R.V3V3" />
    <trace from="R_ES.pin2" to="ESP_R.GPIO18" />
    <trace from="C_ES.pin1" to="ESP_R.GPIO18" />
    <trace from="C_ES.pin2" to="net.GND" />

    {/* status LEDs: PWR is hardwired to V5, STAT is GPIO4 */}
    <led name="LED_PWR" color="red" footprint="0603" schX={-10} schY={14} pcbX={25} pcbY={58} />
    <resistor name="R_PWR" resistance="1k" footprint="0603" schX={-13} schY={14} pcbX={31} pcbY={58} />
    <led name="LED_STAT" color="green" footprint="0603" schX={-10} schY={16} pcbX={25} pcbY={54} />
    <resistor name="R_STAT" resistance="330" footprint="0603" schX={-13} schY={16} pcbX={31} pcbY={54} />
    <trace from="R_PWR.pin1" to="net.V5" />
    <trace from="R_PWR.pin2" to="LED_PWR.anode" />
    <trace from="LED_PWR.cathode" to="net.GND" />
    <trace from="R_STAT.pin1" to="ESP_R.GPIO4" />
    <trace from="R_STAT.pin2" to="LED_STAT.anode" />
    <trace from="LED_STAT.cathode" to="net.GND" />

    {/* ============================================================ */}
    {/* 4x BLDC DRIVER SOCKETS + per-motor phase/hall IO              */}
    {/* socket = PWR strip (3.96mm: V36/GND + phases) above a SIG     */}
    {/* strip (2.54mm: hall pass-through + VR/ZF/EL/M logic)          */}
    {/* ============================================================ */}
    {MOTORS.map((mt) => (
      <>
        <pinheader
          name={`J_D${mt.id}_P`}
      schWidth={0.485}
          pinCount={5}
          pitch="3.96mm"
          gender="female"
          showSilkscreenPinLabels
          pinLabels={DRV_PWR_LABELS}
          schX={14}
          schY={mt.schY}
          pcbX={mt.x}
          pcbY={-2}
        />
        <pinheader
          name={`J_D${mt.id}_S`}
      schWidth={0.58}
          pinCount={11}
          pitch="2.54mm"
          gender="female"
          showSilkscreenPinLabels
          pinLabels={DRV_SIG_LABELS}
          schX={14}
          schY={mt.schY - 4}
          pcbX={mt.x}
          pcbY={-26}
        />
        <Terminal3P
          name={`J_PH_${mt.id}`}
          pinLabels={PH_PINS}
          schX={20}
          schY={mt.schY}
          schHeight={0.4}
          pcbRotation={0}
          pcbX={mt.x}
          pcbY={-60}
        />
        <pinheader
          name={`J_HL_${mt.id}`}
      schWidth={0.58}
          pinCount={5}
          pitch="2.54mm"
          gender="male"
          showSilkscreenPinLabels
          pinLabels={HALL_LABELS}
          schX={20}
          schY={mt.schY - 4}
          pcbX={mt.x}
          pcbY={-44}
        />
        {/* driver power + motor phases (HIGH CURRENT - pour these) */}
        <trace from={`J_D${mt.id}_P.V36`} to="net.V36" />
        <trace from={`J_D${mt.id}_P.GND`} to="net.GND" />
        <trace from={`J_D${mt.id}_P.MA`} to={`J_PH_${mt.id}.MA`} schDisplayLabel={`PH_${mt.id}_A`} />
        <trace from={`J_D${mt.id}_P.MB`} to={`J_PH_${mt.id}.MB`} schDisplayLabel={`PH_${mt.id}_B`} />
        <trace from={`J_D${mt.id}_P.MC`} to={`J_PH_${mt.id}.MC`} schDisplayLabel={`PH_${mt.id}_C`} />
        {/* hall pass-through: module supplies its own 5V to the motor sensors */}
        <trace from={`J_D${mt.id}_S.H5V`} to={`J_HL_${mt.id}.H5V`} schDisplayLabel={`HL_${mt.id}_5V`} />
        <trace from={`J_D${mt.id}_S.HGND`} to={`J_HL_${mt.id}.HGND`} schDisplayLabel={`HL_${mt.id}_G`} />
        <trace from={`J_D${mt.id}_S.HA`} to={`J_HL_${mt.id}.HA`} schDisplayLabel={`HL_${mt.id}_A`} />
        <trace from={`J_D${mt.id}_S.HB`} to={`J_HL_${mt.id}.HB`} schDisplayLabel={`HL_${mt.id}_B`} />
        <trace from={`J_D${mt.id}_S.HC`} to={`J_HL_${mt.id}.HC`} schDisplayLabel={`HL_${mt.id}_C`} />
        {/* logic to the ESP32 */}
        <trace from={`J_D${mt.id}_S.VR`} to={mt.vr} schDisplayLabel={`VR_${mt.id}`} />
        <trace from={`J_D${mt.id}_S.ZF`} to={mt.zf} schDisplayLabel={`ZF_${mt.id}`} />
        <trace from={`J_D${mt.id}_S.EL`} to={mt.el} schDisplayLabel={`EL_${mt.id}`} />
        <trace from={`J_D${mt.id}_S.M`} to={mt.m} schDisplayLabel={`M_${mt.id}`} />
        <trace from={`J_D${mt.id}_S.GND`} to="net.GND" />
      </>
    ))}
  </board>
)

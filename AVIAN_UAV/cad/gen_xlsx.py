"""AVIAN_BOM.xlsx -- the CSV BOM as a formatted, live workbook.

Sheets:
  BOM              every line item, filterable, with the confidence column
                   colour-keyed
  Mass Budget      subsystem roll-up, cross-checked against the assembly
  Propulsion       propulsion and performance closure
  Joint Torque     J1-J6 limits and design torques
  Battery          energy and current
  Endurance        hover power and endurance cases
  Configurations   the 7 delivered configurations
  Serviceability   access, direction, tools, prerequisites
  Legend           what every confidence tag means and what it does not

Totals are FORMULAS, not values, so the workbook recalculates if a line item
is edited.
"""
from __future__ import annotations
import csv
import os

from openpyxl import Workbook
from openpyxl.styles import Font, PatternFill, Alignment, Border, Side
from openpyxl.utils import get_column_letter
from openpyxl.worksheet.table import Table, TableStyleInfo

SRC = "pkg/05_BOM"
FONT = "Arial"

HDR_FILL = PatternFill("solid", fgColor="1F3242")
HDR_FONT = Font(name=FONT, size=10, bold=True, color="FFFFFF")
BODY = Font(name=FONT, size=10)
BOLD = Font(name=FONT, size=10, bold=True)
TITLE = Font(name=FONT, size=13, bold=True, color="1F3242")

CONF_FILL = {
    "VENDOR":      PatternFill("solid", fgColor="DDEBF7"),
    "CALCULATED":  PatternFill("solid", fgColor="E2EFDA"),
    "ESTIMATED":   PatternFill("solid", fgColor="FFF2CC"),
    "ASSUMED":     PatternFill("solid", fgColor="FCE4D6"),
    "PLACEHOLDER": PatternFill("solid", fgColor="F8CBAD"),
}
THIN = Side(style="thin", color="BFBFBF")
BOX = Border(left=THIN, right=THIN, top=THIN, bottom=THIN)


def read(fn):
    with open(os.path.join(SRC, fn)) as f:
        return list(csv.reader(f))


def num(v):
    try:
        if v.strip() == "":
            return v
        f = float(v)
        return int(f) if f == int(f) and "." not in v else f
    except (ValueError, AttributeError):
        return v


def sheet(wb, name, rows, title, note="", widths=None, conf_col=None,
          first=False):
    ws = wb.active if first else wb.create_sheet()
    ws.title = name
    ws["A1"] = title
    ws["A1"].font = TITLE
    ws["A2"] = note
    ws["A2"].font = Font(name=FONT, size=9, italic=True, color="595959")
    r0 = 4

    for j, h in enumerate(rows[0], start=1):
        c = ws.cell(row=r0, column=j, value=h)
        c.font = HDR_FONT
        c.fill = HDR_FILL
        c.alignment = Alignment(horizontal="center", vertical="center",
                                wrap_text=True)
        c.border = BOX
    for i, row in enumerate(rows[1:], start=r0 + 1):
        for j, v in enumerate(row, start=1):
            c = ws.cell(row=i, column=j, value=num(v))
            c.font = BODY
            c.border = BOX
            c.alignment = Alignment(vertical="top", wrap_text=(j == len(row)))
            if conf_col and j == conf_col:
                f = CONF_FILL.get(str(v).strip())
                if f:
                    c.fill = f
                c.alignment = Alignment(horizontal="center")
    ws.freeze_panes = ws.cell(row=r0 + 1, column=1)
    if widths:
        for j, w in enumerate(widths, start=1):
            ws.column_dimensions[get_column_letter(j)].width = w
    else:
        for j in range(1, len(rows[0]) + 1):
            ws.column_dimensions[get_column_letter(j)].width = 18
    return ws, r0, len(rows) - 1


def main():
    wb = Workbook()

    # ---------------- BOM ------------------------------------------------
    bom = read("AVIAN_BOM.csv")
    ws, r0, n = sheet(
        wb, "BOM", bom,
        "AVIAN REV A -- Bill of Materials",
        "Generated from the CAD assembly used for the clearance checks and "
        "the renders, so line-item masses and the mass budget cannot "
        "disagree. Confidence is colour-keyed; see the Legend sheet.",
        widths=[13, 18, 26, 8, 11, 13, 13, 14, 26, 18, 13, 14, 26, 52],
        conf_col=12, first=True)
    last = r0 + n
    tot = last + 2
    ws.cell(row=tot, column=3, value="TOTAL").font = BOLD
    ws.cell(row=tot, column=4, value=f"=SUM(D{r0+1}:D{last})").font = BOLD
    ws.cell(row=tot, column=7, value=f"=SUM(G{r0+1}:G{last})").font = BOLD
    ws.cell(row=tot, column=7).number_format = "0.000"
    ws.cell(row=tot + 1, column=3,
            value="Nominal MTOW (kg), from the assembly").font = BODY
    ws.cell(row=tot + 1, column=7, value=25.753).font = BODY
    ws.cell(row=tot + 1, column=7).number_format = "0.000"
    ws.cell(row=tot + 2, column=3,
            value="Difference (g) -- display rounding only").font = BODY
    ws.cell(row=tot + 2, column=7,
            value=f"=(G{tot}-G{tot+1})*1000").font = BODY
    ws.cell(row=tot + 2, column=7).number_format = "0.00"
    ws.auto_filter.ref = f"A{r0}:N{last}"
    for i in range(r0 + 1, last + 1):
        ws.cell(row=i, column=6).number_format = "0.0000"
        ws.cell(row=i, column=7).number_format = "0.0000"

    # ---------------- Mass budget ----------------------------------------
    mb = read("AVIAN_MASS_BUDGET.csv")
    ws2, r2, n2 = sheet(
        wb, "Mass Budget", mb[:-1],
        "AVIAN REV A -- Mass budget by subsystem",
        "CAD mass measured on the model; Phase 2.7 budget is the frozen "
        "pre-CAD estimate. Every delta is classified in "
        "07_Engineering/MASS_BUDGET.md -- none is unexplained.",
        widths=[24, 16, 22, 14, 18])
    l2 = r2 + n2
    t2 = l2 + 1
    ws2.cell(row=t2, column=1, value="TOTAL").font = BOLD
    for col in (2, 3):
        c = ws2.cell(row=t2, column=col,
                     value=f"=SUM({get_column_letter(col)}{r2+1}:"
                           f"{get_column_letter(col)}{l2})")
        c.font = BOLD
        c.number_format = "0.000"
    ws2.cell(row=t2, column=4, value=f"=(B{t2}-C{t2})*1000").font = BOLD
    ws2.cell(row=t2, column=4).number_format = "0.0"
    ws2.cell(row=t2, column=5, value=f"=SUM(E{r2+1}:E{l2})").font = BOLD
    ws2.cell(row=t2, column=5).number_format = "0.00"
    for i in range(r2 + 1, l2 + 1):
        for col in (2, 3):
            ws2.cell(row=i, column=col).number_format = "0.000"
        ws2.cell(row=i, column=4).number_format = "0.0"
        ws2.cell(row=i, column=5).number_format = "0.00"

    # ---------------- engineering tables ---------------------------------
    TABS = [
        ("TABLE_C_propulsion.csv", "Propulsion",
         "AVIAN REV A -- Propulsion and performance closure",
         "Thrust figures are T-Motor bench data at 100 % throttle, sea "
         "level, static. They are VENDOR, not VERIFIED: no flight test has "
         "been run on this airframe.", [34, 26, 10, 14, 42], 4),
        ("TABLE_D_joint_torque.csv", "Joint Torque",
         "AVIAN REV A -- Manipulator joint schedule",
         "Design torques come from a 123,552-pose worst-orientation sweep "
         "covering gravity, a 90 N contact-force case and an inertial case.",
         [8, 20, 14, 15, 15, 17, 16, 16, 14], 9),
        ("TABLE_E_battery.csv", "Battery",
         "AVIAN REV A -- Battery energy and current",
         "12S2P. Usable energy assumes 80 % depth of discharge.",
         [34, 16, 10, 16], 4),
        ("TABLE_F_endurance.csv", "Endurance",
         "AVIAN REV A -- Endurance estimate",
         "CALCULATED by momentum theory on the vendor coaxial-pair curve. "
         "Static, sea level, no wind, no manoeuvre allowance. Treat the "
         "20 % reserve line as the operationally usable figure.",
         [36, 13, 18, 18, 14, 56], 5),
        ("TABLE_B_cg.csv", "CG",
         "AVIAN REV A -- Centre of gravity by configuration",
         "Measured on the CAD model. The battery detent rows show the trim "
         "authority of the 3-position manual mount.",
         [28, 18, 12, 13, 13, 13, 14], 7),
        ("TABLE_J_configurations.csv", "Configurations",
         "AVIAN REV A -- The seven delivered configurations",
         "Arm state is the J1..J6 command set. All seven are free of static "
         "interference (VERIFIED by B-rep boolean).",
         [20, 26, 12, 24, 20, 30, 18, 12], None),
        ("TABLE_I_serviceability.csv", "Serviceability",
         "AVIAN REV A -- Serviceability matrix",
         "Removal direction and prerequisites are the documented service "
         "sequence; clearances are measured on the CAD model.",
         [28, 20, 16, 22, 34, 34], None),
    ]
    for fn, name, title, note, widths, cc in TABS:
        sheet(wb, name, read(fn), title, note, widths=widths, conf_col=cc)

    # ---------------- legend ---------------------------------------------
    leg = [["Tag", "Means", "Does NOT mean"],
           ["VENDOR", "Taken from a manufacturer data sheet.",
            "That the figure has been reproduced on this airframe. Vendor "
            "bench data is not a flight-test result."],
           ["CALCULATED", "Derived from a documented analysis in "
            "07_Engineering/.",
            "That the analysis has been validated against test or FEA."],
           ["ESTIMATED", "Engineering judgement, no supporting analysis.",
            "Anything more than a considered guess. Expect movement."],
           ["ASSUMED", "A process constant or convention adopted for the "
            "design.", "That it has been confirmed with a supplier."],
           ["PLACEHOLDER", "A dimensionally realistic envelope standing in "
            "for a part not yet selected.",
            "Manufacturer geometry. VERIFY BEFORE FABRICATION."],
           ["VERIFIED", "Measured directly on the CAD geometry.",
            "Physically tested. No hardware exists."],
           ["", "", ""],
           ["NOT USED", "VALIDATED", "Nothing in this package is validated. "
            "No FEA and no physical test has been run on REV A."]]
    wsL, rl, nl = sheet(wb, "Legend", leg,
                        "AVIAN REV A -- Confidence tags",
                        "Read this before quoting any number out of this "
                        "workbook.", widths=[16, 52, 66])
    for i in range(rl + 1, rl + nl + 1):
        t = wsL.cell(row=i, column=1).value
        f = CONF_FILL.get(str(t).strip())
        if f:
            wsL.cell(row=i, column=1).fill = f
        wsL.row_dimensions[i].height = 30

    os.makedirs(SRC, exist_ok=True)
    wb.save(f"{SRC}/AVIAN_BOM.xlsx")
    print(f"wrote {SRC}/AVIAN_BOM.xlsx  ({len(wb.sheetnames)} sheets)")


if __name__ == "__main__":
    main()

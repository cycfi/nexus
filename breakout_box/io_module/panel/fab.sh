kikit panelize \
    --layout 'grid; rows: 3; cols: 2; space: 0mm' \
    --tabs full \
    --cuts vcuts \
    --post 'millradius: 1mm' \
    --framing 'railstb; width: 3mm; space: 3mm;' \
    --tooling '3hole; hoffset: 2.5mm; voffset: 2.5mm; size: 1.5mm' \
    --fiducials '3fid; hoffset: 5mm; voffset: 2.5mm; coppersize: 2mm; opening: 1mm;' \
    --text 'simple; text: JLCJLCJLCJLC; anchor: mt; voffset: 2.5mm; hjustify: center; vjustify: center;' \
    --post 'millradius: 1mm' \
    ../io_module.kicad_pcb io_module_panel.kicad_pcb

kikit fab jlcpcb --no-drc io_module_panel.kicad_pcb .
mv gerbers.zip io_module_gerbers.zip

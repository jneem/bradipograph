ServoDimensions = [22.5, 12.25];
WallThickness = 5;
ExtraOffset = 3;
Height = 4;
ServoScrewDistance = 27.5;
MountScrewDistance = 17;
MountScrewHeight = 8;

$fn=100;

difference() {
    translate([-ServoDimensions[0] / 2 - WallThickness, - ServoDimensions[1] / 2 - WallThickness - ExtraOffset])
    cube([ServoDimensions[0] + 2 * WallThickness, ServoDimensions[1] + WallThickness + ExtraOffset, Height]);
    cube([ServoDimensions[0] + 0.01, ServoDimensions[1] + 0.01, 20], center = true);
    
    // Screw holes for attaching to the servo
    translate([ServoScrewDistance / 2, 0, 0]) cylinder(20, r=0.95, center = true);
    translate([-ServoScrewDistance / 2, 0, 0]) cylinder(20, r=0.95, center = true);
}

translate([ServoDimensions[0] / 2 + WallThickness - MountScrewDistance / 2 - 3, 0, 0])
difference() {
    h = MountScrewHeight + 3;
    w = MountScrewDistance + 6;
    translate([0, -ServoDimensions[1]/2 - WallThickness - ExtraOffset/2, h/2])
    cube([w, ExtraOffset, h], center = true);
    
    translate([MountScrewDistance / 2, 0, MountScrewHeight])
    rotate([90, 0, 0])
    cylinder(50, r=1.5, center = true);

    translate([- MountScrewDistance/2, 0, MountScrewHeight])
    rotate([90, 0, 0])
    cylinder(50, r=1.5, center = true);
}

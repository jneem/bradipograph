ServoDimensions = [22.5, 12.25];
WallThickness = 4;
ExtraOffset = 3;
Height = 4;
ServoScrewDistance = 27.5;
MountScrewDistance = 17;
MountScrewHeight = 8;

$fn=100;

difference() {
translate([-ServoDimensions[0] / 2 - WallThickness, - ServoDimensions[1] / 2 - WallThickness - ExtraOffset])
cube([ServoDimensions[0] + 2 * WallThickness, ServoDimensions[1] + WallThickness + ExtraOffset, Height]);
cube([ServoDimensions[0] + 0.001, ServoDimensions[1] + 0.001, 20], center = true);
    
// Screw holes for attaching to the servo
translate([ServoScrewDistance / 2, 0, 0]) cylinder(20, 0.9, 0.9, center = true);
translate([-ServoScrewDistance / 2, 0, 0]) cylinder(20, 0.9, 0.9, center = true);
}

difference() {
    h = MountScrewHeight + 3;
    translate([(ServoScrewDistance - MountScrewDistance)/2, -ServoDimensions[1]/2 - WallThickness - ExtraOffset/2, h/2])
    cube([MountScrewDistance + 6, ExtraOffset, h], center = true);
    
translate([ServoScrewDistance / 2, 0, MountScrewHeight])
rotate([90, 0, 0])
cylinder(50, r=1.5, center = true);

translate([ServoScrewDistance / 2 - MountScrewDistance, 0, 8])
rotate([90, 0, 0])
cylinder(50, r=1.5, center = true);
}
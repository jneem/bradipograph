BatterySize = [26.2, 16.9];
WallThickness = 1;
BackOffset = 2;
BoxHeight = 15;
ScrewOffset = 2.5;
ChinDepth = 2.5;
PlateWidth = 30;
PlateHeight = 15;
PowerRegulatorWidth = 11;

$fn=100;

module Box() {
    translate([-BatterySize[0] - WallThickness * 1.5, 0, 0])
    cube([BatterySize[0] * 2 + WallThickness * 3, BatterySize[1] + WallThickness * 2 + BackOffset, BoxHeight]);
    
    hull() {
        translate([-BatterySize[0] - WallThickness * 1.5, -ChinDepth, BoxHeight - 0.01])
        cube([BatterySize[0] * 2 + WallThickness * 3, ChinDepth, 0.01]);
        
        translate([-BatterySize[0] - WallThickness * 1.5, 0, BoxHeight - 0.01 - ChinDepth])
        cube([BatterySize[0] * 2 + WallThickness * 3, ChinDepth, 0.01]);
    }
}

module Battery() {
    module Right() {
        translate([WallThickness / 2, WallThickness + BackOffset, WallThickness])
        cube([BatterySize[0], BatterySize[1], 50]);
    }
    
    Right();
    mirror([1,0,0]) Right();

    // Cut the middle piece a little for better access to the screw.
    translate([-BatterySize[0] / 2, WallThickness + BackOffset, WallThickness + BoxHeight - 4])
    cube([BatterySize[0], BatterySize[1], 50]);
}

module ElectricScrewSupport() {
    h = 1.5;
    rotate([90, 0, 0])
    translate([0, 0, h/2]) cylinder(h, r1= 3, r2 = 2, center = true);
}

module Hanger() {
    module Support() {
        translate([0, BackOffset, BoxHeight + ScrewOffset - 2])
        rotate([90, 0, 0])
        linear_extrude(BackOffset)
        offset(r=2) translate([-PlateWidth / 2, 0]) square([PlateWidth, PlateHeight]);
    }

    translate([PlateWidth / 2 - 3, 0, BoxHeight + ScrewOffset + PlateHeight - 3])
    ElectricScrewSupport();
    translate([-PlateWidth / 2 + 3, 0, BoxHeight + ScrewOffset + PlateHeight - 3])
    ElectricScrewSupport();
    
    Support();
}

module ScrewHoles() {
    translate([0, 0, BoxHeight + ScrewOffset])
    rotate([90, 0, 0])
    cylinder(30, r=1.5, center = true);

    translate([PlateWidth / 2 - 3, 0, BoxHeight + ScrewOffset + PlateHeight - 3])
    rotate([90, 0, 0])
    cylinder(30, r=1.45, center = true);

    translate([-PlateWidth / 2 + 3, 0, BoxHeight + ScrewOffset + PlateHeight - 3])
    rotate([90, 0, 0])
    cylinder(30, r=1.45, center = true);
}

module PowerRegulator() {
    h = 5;
    d = 4;
    translate([0, - PowerRegulatorWidth / 2 - WallThickness, h/2])
    difference() {
        cube([d, PowerRegulatorWidth + WallThickness * 2, 5], center = true);
        translate([0, 0, WallThickness])
        cube([d + 0.01, PowerRegulatorWidth, 5.01], center = true);
    }
}

module PowerSwitch() {
    h = 10;
    d = 6;

    translate([-3.5, 0, h / 2])
    difference() {
        cube([7, d, h], center = true);
        translate([0, 0, 2]) cube([3, d + 0.01, h], center = true);
    }
}


module Positive() {
    Box();
    Hanger();

    translate([-BatterySize[0] - WallThickness * 1.5, BatterySize[1] + WallThickness * 2 + BackOffset, 0])
    PowerRegulator();

    translate([-BatterySize[0] - WallThickness * 1.5, 0, 0])
    PowerSwitch();
}

module Negative() {
    Battery();
    ScrewHoles();
}

difference() {
    Positive();
    Negative();
}

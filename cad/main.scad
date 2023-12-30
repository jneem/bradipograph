EyeCenter = [34, -8];
EyeInnerRadius = 11;
EyeRadius = 17;
NoseRadius = 70;
HeadRadius = 60;
StrokeWidth = 5;
HeadTranslation = -30;
ChinTranslation = -38;
StringDiameter = 0.3;

StepperScrewDistance = 35;
StepperScrewOffset = 8;
StepperScrewAngle = 30;

// Stepper driver
DriverWidth = 31.4;
DriverHeight = 35.4;
DriverPosition = [21.5, -65];
DriverScrewPositions = [
    DriverPosition + [2.54, 2.54],
    DriverPosition + [DriverWidth, 0] - [2.54, -2.54],
];

ScrewLocations = [
    [0, HeadRadius + HeadTranslation],
    [0, HeadRadius + HeadTranslation - 17],
    [17, 2.5],
    [-17, 2.5],
    [50.4, 2.5],
    [-50.4, 2.5],
];

use <bradi-wheel.scad>;

$fn=60;

function cat(L1, L2) = [for (i=[0:len(L1)+len(L2)-1]) 
                        i < len(L1)? L1[i] : L2[i-len(L1)]];

function rev(L) = [for (i=[1:len(L)]) L[len(L)-i]];
 
phi = atan2(EyeCenter[1], EyeCenter[0]);
theta = function(r) asin(r / norm(EyeCenter));
s = function(r) sqrt(EyeCenter[0] * EyeCenter[0] + EyeCenter[1] * EyeCenter[1] - r * r);
stringDetachPoint = function(r) [s(r) * cos(theta(r) - phi), -s(r) * sin(theta(r) - phi)];


module strokeCircle(radius, startAngle, endAngle, strokeWidth) {
    step = 0.999;
    points = [for (t = [startAngle:step:(endAngle-0.001)]) [radius * cos(t), radius * sin(t)]];
        
    offset(r=strokeWidth/2)
    polygon(cat(points, rev(0.999 * points)));
}

module ScrewSupport(h) {
    cylinder(h=h, r=2.5);
}
module TightScrewHole() {
    cylinder(h=100, r=1.4, center = true);
}
module LooseScrewHole() {
    cylinder(h=100, r=1.5, center = true);
}


module Mouth() {
    translate([0, HeadRadius * 0.3]) {
        linear_extrude(StrokeWidth)
        strokeCircle(HeadRadius * 1.1, 254, 286, StrokeWidth);
    }
}

module EyeSocket() {
    difference() {
        linear_extrude(StrokeWidth)
        mirror([1, 0]) strokeCircle(EyeRadius + StrokeWidth/2, 170, 310, StrokeWidth);
        
    }
}

module StepperMount() {
    rotate(StepperScrewAngle) {
            translate([-StepperScrewDistance/2, -StepperScrewOffset]) TightScrewHole();
            translate([StepperScrewDistance/2, -StepperScrewOffset]) TightScrewHole();
    }
}

module RightEye() {
    translate(EyeCenter) {
        //Wheel();
        EyeSocket();
    }
}

module Forehead2d() {
    difference() {
        intersection() {
            translate([0, HeadTranslation])
            circle(HeadRadius + StrokeWidth / 2);
        
            translate([-250, 0])
            square([500, 500]);
        }
        
        translate(EyeCenter) circle(EyeRadius);
        mirror([1, 0, 0]) translate(EyeCenter) circle(EyeRadius);

    }
}

module Head() {
    difference() {
        union() {
            difference() {
                union() {
                    linear_extrude(StrokeWidth)
                    Forehead2d();
            
                    // Head outline
                    translate([0, HeadTranslation]) {
                        linear_extrude(StrokeWidth)
                        intersection() {
                            strokeCircle(HeadRadius, 0, 360, StrokeWidth);
                            
                            translate([0, ChinTranslation + 100 - StrokeWidth/2])
                            square([200, 200], center = true);
                        }
                        
                        linear_extrude(StrokeWidth)
                        translate([0, ChinTranslation])
                        offset(r=StrokeWidth/2)
                        square([90, 0.01], center = true);
                    }
                }
        
                translate([0, 0, StrokeWidth / 2]) linear_extrude(StrokeWidth) offset(0.1) Forehead2d();
            }
            for (p=ScrewLocations) {
                translate(p) ScrewSupport(StrokeWidth/2 + 1);
            }
        }
        
        for (p=ScrewLocations) {
            translate(p) TightScrewHole();
        }
    }
}

module Forehead() {
    difference() {
        union() {
            linear_extrude(StrokeWidth / 2 - 1) Forehead2d();
            for(p=ScrewLocations) {
                translate(p) ScrewSupport(StrokeWidth/2 - 1);
            }
        }
        
        for(p=ScrewLocations) {
            translate(p) TightScrewHole();
        }
        translate([0, 0.05, -1]) StringGuide();
        translate([0, -0.2, -1]) StringGuide();

        cube([2*StringDiameter, 2*StringDiameter, 10], center = true);
    }
}


module RightStringGuide() {
    eps = 0.15;
    hull() {
        translate([StringDiameter/2 + eps, eps, 0]) cylinder(h=StrokeWidth, r=eps);
        translate([14, eps, 0]) cylinder(h=StrokeWidth, r=eps);
        translate([14, 5.0, 0]) cylinder(h=StrokeWidth, r=eps);
    }
    
    h = StrokeWidth / 2 - StringDiameter;

    module Guide() {
        hull() {
            translate([StringDiameter/2 + eps, -2*h/3, -eps]) sphere(eps);
            translate([StringDiameter/2 + eps, eps, -h/2]) cylinder(h=h, r=eps, center = true);
            translate([5, eps, -h/2]) cylinder(h=h, r=eps, center = true);

        }
    }
    
    translate([0, 0, StrokeWidth]) Guide();
    

}

module StringGuide() {
    RightStringGuide();
mirror([1, 0, 0]) RightStringGuide();
}

module Nose() {
    module Plank() {
        intersection() {
            rotate([108.5, 0, 0]) translate([0, 20, 0.75]) {
                cube([25, 40, 1.5], center = true);
            }
            translate([0, 0, 50]) cube([100, 100, 100], center = true);
        }
    }
    
    translate([0, -17, 0]) {
        Plank();
        
        difference() {
            union() {
                // Bridge connecting the nose to the eyes
                linear_extrude(StrokeWidth)
                translate([0, -NoseRadius - StrokeWidth/2, 0])
                //offset(r=StrokeWidth/2)
                //square([42, 0.01], center=true);
                strokeCircle(NoseRadius, 72.2, 107.8, StrokeWidth);
                
                // Cylinder supporting the nose
                translate([0, -11, 0]) cylinder(50, r=StrokeWidth/2);
        
                // Bridge connecting the nose to the mouth
                bridge = 28;
        
                linear_extrude(StrokeWidth)
                offset(r=StrokeWidth/2)
                translate([0, - bridge / 2])
                square([0.01, bridge], center=true);
            }
            
            hull() {
                translate([0, 0, 0.1]) Plank();
                translate([0, 15, -1]) Plank();
                translate([0, 0, 20]) Plank();


            }
        }

    }
}

module DriverScrewSupports() {
    module Right() {
        for(p=DriverScrewPositions) {
            translate(p) ScrewSupport(StrokeWidth);
        }
        
        linear_extrude(StrokeWidth)
        offset(r=StrokeWidth/2)
        translate(DriverScrewPositions[0] + [0, -2.5])
        square([0.01, 5], center = true);
    }
    
    Right();
    mirror([1, 0, 0]) Right();
}

module DriverScrewHoles() {
    module Right() {
        for(p=DriverScrewPositions) {
            translate(p) TightScrewHole();
        }
    }
    
    Right();
    mirror([1, 0, 0]) Right();
}


module Positive() {
RightEye();
mirror([1, 0, 0]) RightEye();



Head();
Nose();
Mouth();
    StringGuide();
    
    DriverScrewSupports();
};

module Negative() {
    translate(EyeCenter) StepperMount();
    mirror([1, 0, 0]) translate(EyeCenter) StepperMount();
    DriverScrewHoles();
    
    // Extra screw hole at the bottom, maybe for a battery?
    translate([0, ChinTranslation + HeadTranslation]) TightScrewHole();
    
    for (p=ScrewLocations) {
        translate(p) TightScrewHole();
    }
}

difference() {
    Positive();
    Negative();
}

translate([120, 0, StrokeWidth/2 + 1]) Forehead();

//translate([20, -65, -1]) cube([31.4, 35.4, 1]);



package engine

import "math"

// anchorPadR: nominal "pad radius" used by webbing/pyramid support planning
// (spacing, min length, apex skip). We no longer emit any pad geometry, but
// the supports still need a sensible scale around each anchor — so we keep
// the same mult-by-cls scheme that used to drive the visible pad.
func anchorPadR(s Settings, cls int, floating bool) float64 {
	if floating {
		return s.StrutR * 1.25
	}
	mult := 2.4
	switch cls {
	case 1:
		mult = 1.8
	case 2:
		mult = 2.1
	}
	return s.StrutR * mult
}

func buildSnapMarker(s Settings, point Vec3) *Group {
	g := NewGroup()
	m := newMat(Colors.SnappedToWire, true)
	addOcta(g, point, s.StrutR*0.55, m)
	return g
}

// buildArm returns one human arm geometry oriented along +Z, rooted at the
// origin (shoulder). `reach` is the arm's full extent from shoulder to
// fingertip ish.
func buildArm(s Settings, reach float64, grip string, rand *Rand) *Group {
	strutR := s.StrutR
	g := NewGroup()
	mArm := newMat(Colors.Arm, false)
	upperLen := reach * 0.45
	// upper-arm: cylinder of axis Y, then rotated so Y→Z (rotation.x = pi/2);
	// equivalent to placing it along +Z.
	{
		leaf := NewGroup()
		leaf.Mesh = &Mesh{Tris: makeCylinder(strutR*1.05, strutR*0.95, upperLen, 20), Mat: mArm}
		leaf.Transform.Rotation = QuatFromUnitVectors(V(0, 1, 0), V(0, 0, 1))
		leaf.Transform.Position = V(0, 0, upperLen/2)
		g.Children = append(g.Children, leaf)
	}
	addSphere(g, V(0, 0, upperLen), strutR*1.1, 20, 16, mArm)
	forearmLen := reach * 0.38
	{
		leaf := NewGroup()
		leaf.Mesh = &Mesh{Tris: makeCylinder(strutR*0.95, strutR*0.85, forearmLen, 20), Mat: mArm}
		leaf.Transform.Rotation = QuatFromUnitVectors(V(0, 1, 0), V(0, 0, 1))
		leaf.Transform.Position = V(0, 0, upperLen+forearmLen/2)
		g.Children = append(g.Children, leaf)
	}
	wristZ := upperLen + forearmLen
	addSphere(g, V(0, 0, wristZ), strutR*0.95, 20, 16, mArm)
	// palm: a slightly squashed sphere
	{
		palm := NewGroup()
		palm.Mesh = &Mesh{Tris: makeSphere(strutR*1.05, 20, 16), Mat: mArm}
		palm.Transform.Scale = V(1.05, 0.55, 0.85)
		palm.Transform.Position = V(0, 0, wristZ+strutR*0.6)
		g.Children = append(g.Children, palm)
	}
	fingerBaseZ := wristZ + strutR*1.0
	fingerLen := strutR * 1.1
	fingerR := strutR * 0.25
	closedFist := grip == "clasp" || grip == "rescue" || grip == "forearm"
	if closedFist {
		// knuckle: scaled sphere
		{
			k := NewGroup()
			k.Mesh = &Mesh{Tris: makeSphere(strutR*0.85, 16, 12), Mat: mArm}
			k.Transform.Scale = V(1.1, 0.7, 1.0)
			k.Transform.Position = V(0, strutR*0.15, fingerBaseZ+fingerLen*0.5)
			g.Children = append(g.Children, k)
		}
		// thumb (rotated cylinder)
		{
			th := NewGroup()
			th.Mesh = &Mesh{Tris: makeCylinder(fingerR*1.2, fingerR, fingerLen*0.9, 10), Mat: mArm}
			th.Transform.Position = V(strutR*0.5, strutR*0.2, fingerBaseZ+fingerLen*0.4)
			th.Transform.Rotation = QuatFromAxisAngle(V(0, 0, 1), math.Pi/2.2)
			g.Children = append(g.Children, th)
		}
	} else {
		// open hand: 4 fingers
		for i := 0; i < 4; i++ {
			fg := NewGroup()
			fg.Mesh = &Mesh{Tris: makeCylinder(fingerR, fingerR*0.85, fingerLen, 10), Mat: mArm}
			// rotate Y→Z
			fg.Transform.Rotation = QuatFromUnitVectors(V(0, 1, 0), V(0, 0, 1))
			yOff := (float64(i) - 1.5) * strutR * 0.32
			fg.Transform.Position = V(0, yOff, fingerBaseZ+fingerLen/2)
			g.Children = append(g.Children, fg)
		}
		// thumb
		th := NewGroup()
		th.Mesh = &Mesh{Tris: makeCylinder(fingerR*1.15, fingerR*0.9, fingerLen*0.95, 10), Mat: mArm}
		th.Transform.Position = V(strutR*0.55, strutR*0.1, fingerBaseZ+fingerLen*0.45)
		th.Transform.Rotation = QuatFromAxisAngle(V(0, 0, 1), math.Pi/2.4)
		g.Children = append(g.Children, th)
	}
	_ = rand
	return g
}

// Anchor mirrors what the JS pipeline collects for downstream support
// planning.
type Anchor struct {
	Anchor3D     Vec3
	PadR         float64
	Face         *CubeFace
	Kind         string // "skin" or "floating"
	OwnerStrutID int
	AnchorIdx    int
}

var gripStyles = []string{"formal", "clasp", "rescue", "forearm", "brotherly"}

package main

import "math"

// JointBuild is what buildSkinJoint / buildFloatingJoint return: the geometry
// group, the "clean" interior end (where the strut body should start), and
// the pad radius (informs webbing-support spacing).
type JointBuild struct {
	Mesh    *Group
	JointEnd Vec3
	PadR     float64
}

func buildSkinJoint(s Settings, topo *CubeTopology, anchor Vec3, inwardDir Vec3, face *CubeFace) JointBuild {
	g := NewGroup()
	mAnchor := newMat(Colors.SkinAnchor, true)
	cls := topo.ClassifyBoundary(anchor, s.CubeSize/2)
	strutR := s.StrutR
	mult := 2.4
	switch cls {
	case 1:
		mult = 1.8
	case 2:
		mult = 2.1
	}
	padR := strutR * mult
	padThickness := strutR * 0.45
	filletLen := strutR * 1.6
	inward := inwardDir.Normalize()
	outward := inward.Neg()
	if face != nil {
		outward = face.Normal
	}
	padCenter := anchor.Add(outward.Mul(padThickness * 0.15))
	// pad cylinder along outward
	addCylinder(g, padCenter, outward, padThickness, padR, padR*1.05, 24, mAnchor)
	// fillet: cylinder from filletStart to filletEnd, axis = inward
	filletStart := anchor.Add(outward.Mul(-padThickness * 0.35))
	filletEnd := filletStart.Add(inward.Mul(filletLen))
	filletMid := filletStart.Add(filletEnd).Mul(0.5)
	addCylinder(g, filletMid, inward, filletLen, strutR, padR*0.92, 24, mAnchor)
	return JointBuild{Mesh: g, JointEnd: filletEnd, PadR: padR}
}

func buildFloatingJoint(s Settings, point Vec3) JointBuild {
	g := NewGroup()
	mAnchor := newMat(Colors.FloatingAnchor, true)
	addSphere(g, point, s.StrutR*1.25, 20, 14, mAnchor)
	return JointBuild{Mesh: g, JointEnd: point, PadR: s.StrutR * 1.25}
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

func buildStrut(s Settings, topo *CubeTopology, strut Strut, withHandshake bool, rand *Rand, anchorOut *[]Anchor) *Group {
	strutR := s.StrutR
	half := s.CubeSize / 2
	g := NewGroup()
	a, b := strut.A, strut.B
	dirAB := b.Sub(a).Normalize()
	dirBA := dirAB.Neg()
	aOnSkin := topo.ClassifyBoundary(a, half) >= 1
	bOnSkin := topo.ClassifyBoundary(b, half) >= 1
	var strutColor int
	switch {
	case strut.Lf < 0.999:
		strutColor = Colors.StrutShortened
	case strut.KindGen == "edge":
		strutColor = Colors.StrutEdgeToEdge
	default:
		strutColor = Colors.StrutFull
	}
	matStrut := newMat(strutColor, false)
	var jointA, jointB JointBuild
	if aOnSkin {
		faceA := topo.FaceForBoundaryPoint(a, dirBA, half)
		jointA = buildSkinJoint(s, topo, a, dirAB, faceA)
		if faceA != nil && topo.ClassifyBoundary(a, half) == 1 {
			*anchorOut = append(*anchorOut, Anchor{
				Anchor3D: a, PadR: jointA.PadR, Face: faceA,
				Kind: "skin", OwnerStrutID: strut.ID,
			})
		}
		if strut.SnappedA {
			g.Add(buildSnapMarker(s, a))
		}
	} else {
		jointA = buildFloatingJoint(s, a)
		*anchorOut = append(*anchorOut, Anchor{
			Anchor3D: a, PadR: jointA.PadR, Kind: "floating", OwnerStrutID: strut.ID,
		})
	}
	if bOnSkin {
		faceB := topo.FaceForBoundaryPoint(b, dirAB, half)
		jointB = buildSkinJoint(s, topo, b, dirBA, faceB)
		if faceB != nil && topo.ClassifyBoundary(b, half) == 1 {
			*anchorOut = append(*anchorOut, Anchor{
				Anchor3D: b, PadR: jointB.PadR, Face: faceB,
				Kind: "skin", OwnerStrutID: strut.ID,
			})
		}
		if strut.SnappedB {
			g.Add(buildSnapMarker(s, b))
		}
	} else {
		jointB = buildFloatingJoint(s, b)
		*anchorOut = append(*anchorOut, Anchor{
			Anchor3D: b, PadR: jointB.PadR, Kind: "floating", OwnerStrutID: strut.ID,
		})
	}
	g.Add(jointA.Mesh)
	g.Add(jointB.Mesh)
	cleanA, cleanB := jointA.JointEnd, jointB.JointEnd
	cleanLen := cleanA.DistTo(cleanB)
	cleanMid := cleanA.Add(cleanB).Mul(0.5)
	if !withHandshake {
		addCylinder(g, cleanMid, dirAB, cleanLen, strutR, strutR, 24, matStrut)
		return g
	}
	// Handshake
	offset := (rand.F() - 0.5) * cleanLen * 0.15
	meet := cleanMid.Add(dirAB.Mul(offset))
	distA := cleanA.DistTo(meet)
	distB := cleanB.DistTo(meet)
	armPortion := math.Min(0.5, math.Min(distA*0.55, distB*0.55))
	towardA := dirAB.Neg()
	towardB := dirAB
	shA := meet.Add(towardA.Mul(armPortion))
	shB := meet.Add(towardB.Mul(armPortion))
	segment := func(p1, p2 Vec3) {
		l := p1.DistTo(p2)
		if l < 0.05 {
			return
		}
		m := p1.Add(p2).Mul(0.5)
		d := p2.Sub(p1).Normalize()
		addCylinder(g, m, d, l, strutR, strutR, 24, matStrut)
	}
	segment(cleanA, shA)
	segment(cleanB, shB)
	grip := gripStyles[int(rand.F()*float64(len(gripStyles)))%len(gripStyles)]
	_ = rand.F() // warmA — kept consumed in case future visuals want it
	_ = rand.F() // warmB
	armA := buildArm(s, armPortion, grip, rand)
	armA.Transform.Position = shA
	armA.Transform.Rotation = QuatFromUnitVectors(V(0, 0, 1), towardB)
	// rotateZ((rand-0.5)*0.4) — three.js's rotateZ applies a LOCAL rotation
	// AFTER the existing orientation. Compose: q = q * rotZ.
	armA.Transform.Rotation = QuatMul(armA.Transform.Rotation, QuatFromAxisAngle(V(0, 0, 1), (rand.F()-0.5)*0.4))
	g.Add(armA)
	armB := buildArm(s, armPortion, grip, rand)
	armB.Transform.Position = shB
	armB.Transform.Rotation = QuatFromUnitVectors(V(0, 0, 1), towardA)
	armB.Transform.Rotation = QuatMul(armB.Transform.Rotation, QuatFromAxisAngle(V(0, 0, 1), (rand.F()-0.5)*0.4))
	g.Add(armB)
	return g
}

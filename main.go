package main

import (
	"flag"
	"fmt"
	"log"
	"math/rand/v2"
	"os"
	"runtime"
	"time"

	"github.com/go-gl/gl/v4.1-core/gl"
	"github.com/go-gl/glfw/v3.3/glfw"
	"github.com/go-gl/mathgl/mgl32"
)

const winW, winH = 1100, 760

func init() {
	// GLFW must run on the main OS thread on Mac.
	runtime.LockOSThread()
}

func main() {
	seedFlag := flag.String("seed", "", "hex seed (e.g. 1a2b3c). Empty = random.")
	autoSpin := flag.Bool("spin", true, "auto-rotate while idle")
	stlOnly := flag.String("stl", "", "if set, write STL to this path and exit (no window)")
	mode := flag.String("mode", "debug", "render mode: 'debug' (per-feature mesh, colored) or 'skin' (implicit SDF + marching cubes, single skin)")
	mcRes := flag.Int("res", 240, "skin mode: marching-cubes grid resolution per axis")
	blendK := flag.Float64("k", 20.0, "skin mode: smooth-min sharpness (larger = sharper joints)")
	fillet := flag.Float64("fillet", 0, "skin mode: outer rounded-cube fillet radius (edges + corners). 0 = match strut radius")
	corner := flag.Float64("corner", 0.55, "skin mode: corner shape in [0, 1]. 0 = sharp miter, 0.5 = round (baseline), 1 = flat cut.")
	flag.Parse()

	settings := DefaultSettings()
	var currentSeed uint32
	if *seedFlag != "" {
		s, err := parseHexSeed(*seedFlag)
		if err != nil {
			log.Fatalf("bad seed: %v", err)
		}
		currentSeed = s
	} else {
		currentSeed = uint32(rand.Uint32() & 0xffffff)
	}

	skinParams := DefaultSkinParams()
	skinParams.Resolution = *mcRes
	skinParams.BlendK = *blendK
	skinParams.Fillet = *fillet
	skinParams.Corner = *corner

	build := func(seed uint32) *Group {
		switch *mode {
		case "skin":
			return BuildSkin(settings, seed, skinParams)
		default:
			return BuildScene(settings, seed)
		}
	}
	root := build(currentSeed)

	// Headless mode: write the STL and exit.
	if *stlOnly != "" {
		if err := WriteBinarySTL(*stlOnly, root); err != nil {
			log.Fatalf("stl: %v", err)
		}
		fmt.Printf("wrote %s (seed=%06x)\n", *stlOnly, currentSeed)
		return
	}

	if err := glfw.Init(); err != nil {
		log.Fatalf("glfw init: %v", err)
	}
	defer glfw.Terminate()
	glfw.WindowHint(glfw.ContextVersionMajor, 4)
	glfw.WindowHint(glfw.ContextVersionMinor, 1)
	glfw.WindowHint(glfw.OpenGLProfile, glfw.OpenGLCoreProfile)
	glfw.WindowHint(glfw.OpenGLForwardCompatible, glfw.True)
	glfw.WindowHint(glfw.Samples, 4)

	win, err := glfw.CreateWindow(winW, winH, "Marble Handshake Lattice — Go/OpenGL", nil, nil)
	if err != nil {
		log.Fatalf("create window: %v", err)
	}
	win.MakeContextCurrent()
	if err := gl.Init(); err != nil {
		log.Fatalf("gl init: %v", err)
	}
	glfw.SwapInterval(1)
	gl.Enable(gl.MULTISAMPLE)

	bg := [3]float32{0x1a / 255.0, 0x1a / 255.0, 0x1c / 255.0}
	r, err := NewRenderer(bg)
	if err != nil {
		log.Fatalf("renderer: %v", err)
	}
	r.Upload(root)

	// Interaction state mirrors the JS: rotX clamped to ±1.3, rotY free.
	rotX := float32(0.3)
	rotY := float32(0.6)
	spinning := *autoSpin
	dragging := false
	lastMX, lastMY := 0.0, 0.0
	camPos := mgl32.Vec3{4.2, 2.8, 5.0}

	regen := func() {
		currentSeed = uint32(rand.Uint32() & 0xffffff)
		root = build(currentSeed)
		r.Upload(root)
		fmt.Printf("regenerated, seed=%06x\n", currentSeed)
	}

	saveSTL := func() {
		path := fmt.Sprintf("lattice-%06x.stl", currentSeed)
		if err := WriteBinarySTL(path, root); err != nil {
			fmt.Fprintf(os.Stderr, "stl save failed: %v\n", err)
			return
		}
		fmt.Printf("wrote %s\n", path)
	}

	win.SetMouseButtonCallback(func(_ *glfw.Window, button glfw.MouseButton, action glfw.Action, _ glfw.ModifierKey) {
		if button != glfw.MouseButtonLeft {
			return
		}
		if action == glfw.Press {
			dragging = true
			lastMX, lastMY = win.GetCursorPos()
		} else if action == glfw.Release {
			dragging = false
		}
	})
	win.SetCursorPosCallback(func(_ *glfw.Window, x, y float64) {
		if !dragging {
			return
		}
		rotY += float32(x-lastMX) * 0.008
		rotX += float32(y-lastMY) * 0.008
		if rotX > 1.3 {
			rotX = 1.3
		}
		if rotX < -1.3 {
			rotX = -1.3
		}
		lastMX, lastMY = x, y
	})
	win.SetKeyCallback(func(_ *glfw.Window, key glfw.Key, _ int, action glfw.Action, _ glfw.ModifierKey) {
		if action != glfw.Press {
			return
		}
		switch key {
		case glfw.KeyEscape, glfw.KeyQ:
			win.SetShouldClose(true)
		case glfw.KeyR:
			regen()
		case glfw.KeyS:
			saveSTL()
		case glfw.KeySpace:
			spinning = !spinning
			if spinning {
				fmt.Println("auto-rotate on")
			} else {
				fmt.Println("auto-rotate off")
			}
		}
	})
	win.SetFramebufferSizeCallback(func(_ *glfw.Window, w, h int) {
		gl.Viewport(0, 0, int32(w), int32(h))
	})

	fmt.Println("controls: drag = rotate · space = pause · R = regen · S = save STL · ESC = quit")
	fmt.Printf("seed=%06x\n", currentSeed)
	lastTime := time.Now()
	for !win.ShouldClose() {
		now := time.Now()
		dt := float32(now.Sub(lastTime).Seconds())
		lastTime = now
		if spinning && !dragging {
			rotY += 0.4 * dt
		}
		fbw, fbh := win.GetFramebufferSize()
		r.Draw(fbw, fbh, rotX, rotY, camPos)
		win.SwapBuffers()
		glfw.PollEvents()
	}
}

func parseHexSeed(s string) (uint32, error) {
	var v uint64
	if _, err := fmt.Sscanf(s, "%x", &v); err != nil {
		// fall back to decimal
		if _, err2 := fmt.Sscanf(s, "%d", &v); err2 != nil {
			return 0, err
		}
	}
	return uint32(v), nil
}

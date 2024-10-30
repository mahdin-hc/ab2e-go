package main

import (
	"log"
	"math"
	"time"
	"unsafe"
	"github.com/ByteArena/box2d"
	"main/gl"
	"main/glut"
	"main/loader"
	"main/utils"
)

const (
	scale       = 30
	fps         = 60
	stepTime    = 1.0 / fps
	numSegments = 30
)

var winWidth    = 600
var winHeight   = 400

var (
	staticColor    = [3]float32{0.498, 0.898, 0.498}
	dynamicColor   = [3]float32{0.898, 0.698, 0.698}
	kinematicColor = [3]float32{0.498, 0.498, 0.898} 

	world       box2d.B2World
	angle       float32 = 0.0
	aspectRatio float32
	circleVBO   uint32
	circleData  []float32
)

func main() {
	glut.Init()
	if err := gl.Init(); err != nil {
		log.Fatalf("Failed to initialize OpenGL: %v", err)
	}
	glut.InitDisplayMode(glut.DOUBLE | glut.RGB | glut.DEPTH)
	glut.InitWindowSize(winWidth, winHeight)
	glut.CreateWindow("Box2D with OpenGL")

	gl.ClearColor(0.0, 0.0, 0.0, 1.0)
	gl.Enable(gl.DEPTH_TEST)
	gl.Enable(gl.BLEND)
	gl.BlendFunc(gl.SRC_ALPHA, gl.ONE_MINUS_SRC_ALPHA)


	aspectRatio = float32(winWidth) / float32(winHeight)

	world = box2d.MakeB2World(box2d.B2Vec2{X: 0, Y: 10})
	// initCircleVBO()

	glut.DisplayFunc(display)
	glut.ReshapeFunc(reshape)
	glut.IdleFunc(idle)

	initBox2D()
	glut.MainLoop()
}

func initBox2D() {
	jsonStr, _ := utils.ReadFile("scene.json")
	scene, _ := loader.DecodeScene(jsonStr)
	loader.LoadScene(&world, *scene)
}

func initCircleVBO() {
	circleData = make([]float32, (numSegments+2)*2) // Circle center + circle segments
	gl.GenBuffers(1, &circleVBO)
	gl.BindBuffer(gl.ARRAY_BUFFER, circleVBO)
	gl.BufferData(gl.ARRAY_BUFFER, len(circleData)*4, unsafe.Pointer(&circleData[0]), gl.STATIC_DRAW)
}

func reshape(width, height int) {
	winWidth = width
	winHeight = height
	gl.Viewport(0, 0, int32(width), int32(height))
	gl.MatrixMode(gl.PROJECTION)
	gl.LoadIdentity()
	gl.Ortho(-float64(width)/2, float64(width)/2, -float64(height)/2, float64(height)/2, -1, 1)
	gl.MatrixMode(gl.MODELVIEW)
	gl.LoadIdentity()
}

func display() {
	gl.Clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT)

	renderWorld()
	
	drawAxis()

	glut.SwapBuffers()
}

func idle() {
	start := time.Now()

	world.Step(stepTime, 8, 3)

	angle += 0.5
	if angle > 360 {
		angle -= 360
	}

	glut.PostRedisplay()

	elapsed := time.Since(start)
	sleepTime := time.Second/time.Duration(fps) - elapsed
	if sleepTime > 0 {
		time.Sleep(sleepTime)
	}
}

func renderWorld() {
	for body := world.GetBodyList(); body != nil; body = body.GetNext() {
		switch body.GetType() {
		case box2d.B2BodyType.B2_staticBody:
			gl.Color4f(staticColor[0], staticColor[1], staticColor[2], 0.4)
		case box2d.B2BodyType.B2_dynamicBody:
			gl.Color4f(dynamicColor[0], dynamicColor[1], dynamicColor[2], 0.4)
		case box2d.B2BodyType.B2_kinematicBody:
			gl.Color4f(kinematicColor[0], kinematicColor[1], kinematicColor[2], 0.4)
		}

		for fixture := body.GetFixtureList(); fixture != nil; fixture = fixture.GetNext() {
			shape := fixture.GetShape()
			shapeType := shape.GetType()

			switch shapeType {
			case box2d.B2Shape_Type.E_circle:
				circleShape := shape.(*box2d.B2CircleShape)
				position := body.GetPosition()
				radius := circleShape.GetRadius() * scale
				drawCircle(float32(position.X*scale), -float32(position.Y*scale), float32(radius))
			case box2d.B2Shape_Type.E_polygon:
				polygonShape := shape.(*box2d.B2PolygonShape)
				points := make([][2]float32, polygonShape.M_count)
				for i := 0; i < polygonShape.M_count; i++ {
					vertex := body.GetWorldPoint(polygonShape.M_vertices[i])
					points[i] = [2]float32{float32(vertex.X * scale), -float32(vertex.Y * scale)}
				}
				drawPolygon(points)
			case box2d.B2Shape_Type.E_edge:
				edgeShape := shape.(*box2d.B2EdgeShape)
				v1 := body.GetWorldPoint(edgeShape.M_vertex1)
				v2 := body.GetWorldPoint(edgeShape.M_vertex2)
				drawLine(float32(v1.X*scale), -float32(v1.Y*scale), float32(v2.X*scale), -float32(v2.Y*scale))
			case box2d.B2Shape_Type.E_chain:
				chainShape := shape.(*box2d.B2ChainShape)
				numVertices := chainShape.M_count
				vertices := make([][2]float32, numVertices)
				for i := 0; i < numVertices; i++ {
					vertex := body.GetWorldPoint(chainShape.M_vertices[i])
					vertices[i] = [2]float32{float32(vertex.X * scale), -float32(vertex.Y * scale)}
				}
				drawChain(vertices)
			}
		}
	}
}

func drawAxis() {
    gl.Color4f(0.0, 1.0, 0.0, 0.4) // Set color to white for the axes
    gl.LineWidth(0.5) // Optional: Set the line width for better visibility

    // Draw the x-axis
    gl.Begin(gl.LINES)
    gl.Vertex2f(-float32(winWidth)/2, 0) // Start of the x-axis
    gl.Vertex2f(float32(winWidth)/2, 0)  // End of the x-axis
    gl.End()

    // Draw the y-axis
    gl.Begin(gl.LINES)
    gl.Vertex2f(0, -float32(winHeight)/2) // Start of the y-axis
    gl.Vertex2f(0, float32(winHeight)/2)  // End of the y-axis
    gl.End()
}


func drawCircle(x, y, radius float32) {
    angleIncrement := 2 * math.Pi / numSegments
    gl.Begin(gl.TRIANGLE_FAN)
    gl.Vertex2f(x, y)
    for i := 0; i <= numSegments; i++ {
        angle := angleIncrement * float64(i)
        vX := x + radius*float32(math.Cos(angle))
        vY := y + radius*float32(math.Sin(angle))
        gl.Vertex2f(vX, vY)
    }
    gl.End()
}


func drawPolygon(vertices [][2]float32) {
	if len(vertices) == 0 {
		return
	}

	gl.Begin(gl.POLYGON)
	for _, v := range vertices {
		gl.Vertex2f(v[0], v[1])
	}
	gl.End()
}

func drawLine(x1, y1, x2, y2 float32) {
	gl.Begin(gl.LINES)
	gl.Vertex2f(x1, y1)
	gl.Vertex2f(x2, y2)
	gl.End()
}

func drawChain(vertices [][2]float32) {
	if len(vertices) < 2 {
		return
	}

	gl.Begin(gl.LINE_STRIP)
	for _, v := range vertices {
		gl.Vertex2f(v[0], v[1])
	}
	gl.End()
}

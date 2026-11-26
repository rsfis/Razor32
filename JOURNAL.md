---
title: "Fiscion Razor32"
author: "rsfiscina01"
description: "Portable Foldable Computer"
created_at: "2026-05-18"
---

# 2026-11-25: Kernel Development II

**Total time spent: 15 hours**

From 21. Nov 2026 until 25. Nov 2026

*This was not logged, because I forgot.
*This part was made entirely using Claude and ChatGPT, because rasterization and 3D calculus from scratch running on a CPU is pretty difficult.

## 3D
During these 4 days, I've been working on a 3D rendering system for the kernel.
It was entirely inspired on the Nintendo 64 style, and it has pretty similar limitations.

### Limits:
- Textures max 128x128 (recommended is 64x64 or 32x32)
- Models Skeletons max amount of bones: 48
- Scene max recommended triangles: 3500 triangles
- Intern Resolution: 240x160
- Output Upscaled Resolution: 480x320
- Models accepted: .rzobj (convertion tool in firmware folder)
- No lightning

### How does it work
#### Basics
The rendering system works with 2 engines that share the output to the screen: A 2D compositor that unifies the image and comes everything together to blit to the screen, and a 3D raster that rasterizes the 3D scene and blit the pixels to the 2D compositor. The 2D compositor accesses the TFT Screen memory using DMA - Direct Memory Access, which promotes a really fast memory access, optimizing the blits and screen's wait time.

The zBuf is the Z-Buffer, a buffer that stores the depth information, stored in the ESP32's internal RAM to make the access to the variable during the rasterization faster.

The colorBufA and colorBufB are the buffers tha store color information along the screens pixels. They work andd serve as assyncronous double buffering.

The 3D raster renders the scene on a internal resolution of 240x160 and sends the image upscaled to fit in the screen's resolution 480x320.

A 3D scene is composed of a own 3D models file type .rzobj, containing mesh + uv + albedo_texture + skinning + skeleton animation, which can be obtained by a python script run on a Windows or Linux computer, that converts a .glb to this .rzobj.

The rasterization loop goes through these steps:
 - → Camera → View/Proj → MVP per mesh
 - → prepare(): model culling (frustrum and occlusion) and meshlets, backface cone, drawing order and Z-Buffering
 - → meshlets' skinning computation if visible
 - → clear color and Z
 - → draw(): projects meshlets → polygon culling → clip near/far → rasterize triangle
 - → color buffer 240×160
 - → upscale
 - → send image to 2D compositor and finally to screen
The camera has position, target, yaw/pitch and FOV. Supports orbit and freecam modes.

#### Meshlets and textures
Every model's mesh is cut into meshlets (a chunk of a mesh) of 64 vertices and 126 triangles to optimize the culling and divide to multicore, with a own AABB - Technique that uses cuboid (3D) bounding boxes whose edges stay parallel to the coordinate axes (X, Y, and Z) to quickly test for intersections, indices and a normals cone (to discard backface in chunks).

The rasterization system has a baked bilinear filtering (N64 style, that blurs the textures to make them smooth when streched): During the loading, the texture is read, expanded and gets a bilinear 3 point filtering applied to optimize the image generation. On real-time, the raster only performs point sampling: one reading per pixel, zero filter multiplications.

For optimization purposes, the 2D composer and 3D raster only calculates the part of the image that changes (Dirty rects).

Before rasterizing, the system rejects everything that is not important before spending performance on maths, using bounding boxes, backface, backface culling, frustrum culling per triangle, occlusion culling.

The rasterTri only fills the pixels, colours and depth (the culling and projection part was already done before it).

### How to use
First, initiate the 3D renderer on the lua VM using:
```lua
local ok = Init3DRenderer({ renderDiv = 2, dualCore = false })
print("[3D] Init3DRenderer =>", tostring(ok))
assert(ok, "Init3DRenderer failed")
```

Then, load configure the renderer (this activates all the optimization tricks):
```lua
OS_SetMaxFramerate(0) -- Turns off the framerate limit

Render3D_SetOpt("modelcull", true);
Render3D_SetOpt("cone", true);
Render3D_SetOpt("hiz", true);
Render3D_SetOpt("hiztri", true);
Render3D_SetOpt("hizspan", true);
```

#### Loading and Using Models
```lua
local horse = Model3D("/home/apps/3d_test/models/horse.rzobj") -- loads from file
horse:createInstance({positionX, positionY, positionZ}, {scaleX, scaleY, scaleZ}) -- creates a instance of this model
horse:setCull("pos") -- sets the culling mode. can be "none", "pos", "neg" (inverts the culling face)
horse:SetFiltering(false) -- activates or not the filtering
horse.instances[0]:playAnim(0, true, 1.0, 0.2) -- plays the animation of index 0 for the instance of index 0, looping = true, velocity = 1.0, crossFade = 0.2
horse:updateAnim(dt) -- Updates the skeleton animation based on deltaTime

local sky = Model3D("/home/apps/3d_test/models/sky.rzobj") -- loads skybox from file
sky:createInstance({0, 0, 0}, {100, 100, 100})
sky:SetFiltering(true) -- activates the texture filtering
sky:setCull("none") -- disables the skybox model culling
```

#### Configure the camera and motion
```lua
Camera3D_SetFov(60) -- Sets FOV
Camera3D_SetNearFar(0.1, 100) -- Sets near/far limtits
Camera3D_SetPosition(CAM_START[1], CAM_START[2], CAM_START[3]) -- defines position
Camera3D_SetYawPitch(math.pi, 0) -- sets the yaw as math.pi and pitch as 0
Camera3D_Freecam() -- Updates the camera VIEW based on the freecam mode.

OS_SetKeyboardDelay(0) -- Disables keyboard delay timer antiflood
```

#### Draw 3D Scene
```
Begin3DFrame() -- begins 3D drawing
  horse:draw() -- adds horse model to the drawing queue
  sky:draw() -- adds skybox model to the drawing queue
  Render3D(false)        -- only rasterizes
Composite3DToFrame()     -- copies the 3D image to 2D compositor frame and ends 3D drawing

-- UI goes here

Display_UpdateScreen() -- Updates the screen frame and switch buffers
```

### 3D Example with Free Camera
```lua
local gothambold_font = Font("/sys/fonts/Gotham Bold.bin")

-- Camera livre
local MOVE_SPEED  = 3.0     -- unidades por segundo (W/A/S/D horizontal, Q/E sobe/desce)
local MOUSE_SENS  = 0.003   -- radianos por contagem do mouse
local CAM_START   = {0.0, 0.0, 3.0}
local DT_MAX, DT_MIN = 0.100, 0.0001

-- FPS Counter
local fps_counter = 0
local fps_frames = 0
local fps_last_update = os.clock()

OS_SetMaxFramerate(0)

print("[test_3d] start")

local ok = Init3DRenderer({ renderDiv = 2, dualCore = false })
print("[test_3d] Init3DRenderer =>", tostring(ok))
assert(ok, "Init3DRenderer failed")

Render3D_SetOpt("modelcull", true);
Render3D_SetOpt("cone", true);
Render3D_SetOpt("hiz", true);
Render3D_SetOpt("hiztri", true);
Render3D_SetOpt("hizspan", true);

print("[test_3d] load link")
local link = Model3D("/home/apps/3d_test/models/link_anim.rzobj")
link:createInstance({0.0, -1.4, 0.0}, {0.3, 0.3, 0.3})
link:setCull("pos")
link:SetFiltering(false)
link.instances[0]:playAnim(1, true, 1.0, 0.2)

print("[test_3d] load epona")
local epona = Model3D("/home/apps/3d_test/models/epona.rzobj")
epona:createInstance({2.0, -1.4, 0.0}, {0.3, 0.3, 0.3})
epona:setCull("pos")
epona:SetFiltering(false)
epona.instances[0]:playAnim(0, true, 1.0, 0.2)

print("[test_3d] load map")
local map = Model3D("/home/apps/3d_test/models/kokiri.rzobj") --field ou kokiri
map:createInstance({3, -1.4, 1}, {0.2, 0.2, 0.2}) --0.2 ou 1.0
map:setCull("pos")

print("[test_3d] load skybox")
local sky = Model3D("/home/apps/3d_test/models/sky.rzobj")
sky:createInstance({0, 0, 0}, {100, 100, 100})
sky:setCull("none")

Camera3D_SetFov(60)
Camera3D_SetNearFar(0.1, 100)
-- yaw = pi: olhando para -Z (em direcao a origem, como a antiga camera orbital)
Camera3D_SetPosition(CAM_START[1], CAM_START[2], CAM_START[3])
Camera3D_SetYawPitch(math.pi, 0)
Camera3D_Freecam()

-- sem atraso anti-flood: o movimento precisa ser continuo enquanto a tecla esta segura
OS_SetKeyboardDelay(0)

print("[test_3d] enter loop")

local last = os.clock()

local function clamp(v, a, b)
    if v < a then return a end
    if v > b then return b end
    return v
end

while true do
    local now = os.clock()
    local dt = clamp(now - last, DT_MIN, DT_MAX)
    last = now

    link:updateAnim(dt)
    epona:updateAnim(dt)

    -- ===== olhar com o mouse =====
    -- mouse p/ direita => gira p/ direita (yaw diminui); mouse p/ cima => olha p/ cima (pitch aumenta)
    local mdx, mdy = OS_GetMouseDelta()
    if mdx ~= 0 or mdy ~= 0 then
        Camera3D_Rotate(-mdx * MOUSE_SENS, -mdy * MOUSE_SENS)   -- pitch e limitado a +-1.57 no firmware
    end

    -- ===== mover com WASD (+ Q/E para descer/subir) =====
    local fwd, strafe, vert = 0, 0, 0
    if OS_IsKeyPressed("W") then fwd = fwd + 1 end
    if OS_IsKeyPressed("S") then fwd = fwd - 1 end
    if OS_IsKeyPressed("D") then strafe = strafe + 1 end
    if OS_IsKeyPressed("A") then strafe = strafe - 1 end
    if OS_IsKeyPressed("E") then vert = vert + 1 end
    if OS_IsKeyPressed("Q") then vert = vert - 1 end

    if fwd ~= 0 or strafe ~= 0 or vert ~= 0 then
        local yaw, pitch = Camera3D_GetYawPitch()
        local cp = math.cos(pitch)
        -- frente (com pitch: voa na direcao do olhar) e direita (sempre horizontal)
        local fx, fy, fz = cp * math.sin(yaw), math.sin(pitch), cp * math.cos(yaw)
        local rx, rz = -math.cos(yaw), math.sin(yaw)

        local step = MOVE_SPEED * dt
        Camera3D_Move(
            (fx * fwd + rx * strafe) * step,
            (fy * fwd + vert) * step,
            (fz * fwd + rz * strafe) * step
        )
    end

    Camera3D_Freecam()   -- recalcula o alvo a partir de posicao + yaw/pitch

    Display_ClearScreen()
    
    Begin3DFrame()
    link:draw()
    epona:draw()
    map:draw()
    sky:draw()
    Render3D(false)          -- so rasteriza, nao envia
    Composite3DToFrame()     -- copia o 3D (2x) para o buffer 2D

    fps_frames = fps_frames + 1

    local fps_now = os.clock()
    local elapsed = fps_now - fps_last_update

    if elapsed >= 1.0 then
        fps_counter = fps_frames / elapsed
        fps_frames = 0
        fps_last_update = fps_now
    end

    gothambold_font:drawString(
        tostring(math.floor(fps_counter + 0.5)) .. " FPS",
        0, 0, 20, 255, 255, 255
    )

    Display_UpdateScreen()
end
```

### Result
This is the result of a 3D scene with 3400 triangles, composed of 2 skinned models, one map and one skybox.

![result3d.jpeg](https://cdn.hackclub.com/01a215f5-82ce-7475-bad7-08780749cbe0/result3d.jpeg)

# 2026-11-25: Kernel Development I

**Total time spent: 3 hours**

From 16. Nov. 2026 until 20. Nov. 2026

*ps: I didn't log everything to Hackclub Lapse, because I forgot. But I swear it was made by me.

Now I have all the hardware assembled required, we can proceed with the kernel/firmware development.

# Terms Definitions
- #define Kernel "The ArduinoIDE firmware compiled for the ESP32 that controls the execution of the APPs, the screen and keyboard, and manages Wifi and Bluetooth."
(I know the actual kernel of a ESP32 is the FreeRTOS, but since the firmware actually controls and manages basically everything, even the apps, usage of the pins, I dont care. It will be called the kernel)
- #define APP(s) "A LUA script interpreted by the Kernel, which runs on a Lua VM, that calls from loading assets and drawing functions to keyboard checks."

# Tasks
Since the ESP32 has 2 cores, I can split the tasks with priorities using the FreeRTOS library and the xTaskCreatePinnedToCore.
For now there are the following tasks:
"Kernel" -> The main task. It is the kernel itself, as defined above. Manages the current Lua VM and keyboard, mainly shortcuts, like changing keyboard layouts, capslock and quit current app and return to HOME. And it also manages Toast notifications.
"Wifi" -> Manages Wifi and HTTPS requests
"BLE" -> Manages Bluetooth and wireless mouse connection
"R3DBLIT" -> Manages the 3D drawing and blit to the screen (Explained later)
"r3d_raster" -> Rasterizes the 3D scene (Explained later)
"LUA_APP" -> The current App opened

As I've already documented, the kernel already has basic OS functions, Sprites, Texts/Fonts glyphs, basic shapes drawing functions.

# New Features
## Bluetooth Mouse Support
For this computer, I decided to add a wireless mouse support. So, when you press a key on the keyboard, the kernel activates the bluetooth and start searching for BLE Mouse Devices nearby. When it finds, a Toast notification appears, so you can confirm the connection or deny it and put that device on a blacklist, cleaned when bluetooth is deactivated.
The Apps can read and check by the functions OS_GetMousePosition, OS_GetMouseScrollDelta, OS_GetMouseDelta, OS_IsMouse1Pressed and OS_IsMouse2Pressed.
![ble_1.jpeg](https://cdn.hackclub.com/01a215aa-99ae-7497-9909-990d164a153d/ble_1.jpeg)
![ble_2.jpeg](https://cdn.hackclub.com/01a215aa-9a12-793f-a2f8-53bc46f6b26d/ble_2.jpeg)

## Wifi
The kernel can search wifi networks, connect to them and forget them using, of course, the kernel exported functions to the Lua VM. For now, there are no HTTP Requests method, but it will be created later.

## HOME Page
We can now enter our password into the log in screen, enter the HOME and move the mouse, connect to the WIFI and Bluetooth, everything based on a GUI.

# 2026-11-25: PCB Issues

**Total time spent: 2 hours**

My PCB didn't work.

The keyboard was missing diodes, and the current flow was returning to the origin, activating nearby button's X and Y pin.

So, I redesigned it and it came like this:
![image.png](https://cdn.hackclub.com/01a21571-3c25-7d06-820f-a69426bfd16c/image.png)
I also checked it with my physics teacher, which is formed on Computer Science.

Then, I had to solder everything again, and it came perfect. Everything works beautifully: The screen reacts to the ESP32 commands, and the keyboard is checked magnificiently.
I can even type down my password on the system and log in!
![devlog3.jpeg](https://cdn.hackclub.com/01a21576-7ae6-7b3b-9d26-f29443e0b6ba/devlog3.jpeg)
![devlog2.jpeg](https://cdn.hackclub.com/01a21576-7c86-7138-b3bb-1c765faf6241/devlog2.jpeg)
![devlog1.jpeg](https://cdn.hackclub.com/01a21576-7b27-7f20-8ca2-90480b5b9d64/devlog1.jpeg)

# 2026-06-07: Solved new unified board DRC isssues, Prices and Nets

**Total time spent: 2.07 hours**

During these 2 days, I've been solving the board's DRC errors (Nets, unconnected traces and this king of things) and attaching nets to the traces to be more organized.
I also fixed the prices, cuz they were too high.
First: Removed PCBA. Im handsoldering everything to learn.
Second: Searched the components at aliexpress to reduce costs
Third: Removed the powerbank costs to hackclub. The company won't pay for the powerbank.

Lapses at:
https://lapse.hackclub.com/timelapse/hVn5yxIEUJEf
https://lapse.hackclub.com/timelapse/W5_5n5CTUVG4

![aliexpress.png](https://cdn.hackclub.com/019ea2b7-f2d4-7ab7-8b5e-fdc6fb2e937e/aliexpress.png)
![jlc.png](https://cdn.hackclub.com/019ea2b8-041c-721c-b5a9-64914b0c1a9a/jlc.png)
![image.png](https://cdn.hackclub.com/019ea2b8-4566-7fc9-b82a-cf74fec8e1da/image.png)

# 2026-06-05: Unified the 2 PCBS

**Total time spent: 2 hours**

During these 2 days, as the reviewer asked, I unified the 2 PCBS into one PCB.

Before: 2 pcbs with 2 layers
After: 1 pcb with 4 layers

Im pretty happy with the result.

This decision was important, because this way, the project is now pretty thinner, since the second floor is gone.


First, I put all components into another pcb sketch, and started trying to fit everything.
I wanted to make a 2 layer unified pcb, but it just didn't fit, because there is too many tracks.
So, I added 2 more inner tracks and started routing just like the other pcbs.

This is the final routing and displacement:
![image.png](https://cdn.hackclub.com/019e9a14-9995-791b-8da9-a8cd481a6d7d/image.png)
F.Cu
![image.png](https://cdn.hackclub.com/019e9a14-ef15-776e-ab54-2a114d270246/image.png)
In.1.Cu
![image.png](https://cdn.hackclub.com/019e9a15-2ac7-7f65-a99d-7dbcc88320c5/image.png)
In.2.Cu
![image.png](https://cdn.hackclub.com/019e9a15-5ecc-78b3-8588-41ce45a9a9ad/image.png)
B.Cu
![image.png](https://cdn.hackclub.com/019e9a15-8c56-7ef0-9d29-8406d29d2e8c/image.png)
3D render front
![image.png](https://cdn.hackclub.com/019e9a15-c1cd-702c-8423-ca17017fb9ae/image.png)
3D render back
![image.png](https://cdn.hackclub.com/019e9a16-30f5-767a-8891-511ec2bbf898/image.png)

https://lapse.hackclub.com/timelapse/RG1gdj3PHxtt

# 2026-06-03: 5/18/2026 - Transference from Blueprint to Forge program

**Total time spent: 0.1 hours**

**Total time spent: 0.1 hours**

Time spent: 0.1h  

After 2 months of waiting, I decided to transfer Blueprint to Forge.

Now, Im going to wait anxiously to the final review.

hahahahahahahahahahahahahahaha

![image](https://blueprint.hackclub.com/user-attachments/blobs/proxy/eyJfcmFpbHMiOnsiZGF0YSI6MTYwNjU1LCJwdXIiOiJibG9iX2lkIn19--524e89be46406af7e8b07ccf29cad5bd0bef55a9/image.png)

# 2026-05-28: 2026-05-18: 1/03/2026 - The idea

**Total time spent: 1.0h_**

**Total time spent: 1.0h_**

_Time spent: 1.0h_  

Hi! This is the first journal entry of the Rian's MicroComputer.
This is my first hardware project, and it was in these 3 weeks that I learned how to use KiCad and 3D rendering (thats why it has 12h only for pcbs and 22h only for 3D rendering from scratch using pixels, no 3D lib behind it)

I always was addicted to computers, mainly in how they work. Recently I've been addicted to OS's and PCBs. Then, I wanted so much to do something that unified these 2 addictions. So, from 3rd January to 5th January, I had an idea: what if I made my own computer?

I started studying my idea, and tried to create the first image on my head. I wanted it to be like a Nintendo 3DS - portable, foldable, small, but instead of an analog stick, it would have a keyboard.

I knew that I would need a screen and a processor, but which?
For the processor, I thought about an Arduino micro controller, and for the screen, something that communicates with it using GPIO pins: TFT Screens! So I bought one 3.5" screen at Aliexpress. But when I tested the arduino I already had with the screen, it was SO SLOW. Then, I thought: Why not a Raspberry PI? And the answer is: I want to make the Operational System too! And it is so complicated to do with a real micro computer. So, lets simplify things: I found an Arduino, but more powerful, stronger and smaller: ESP32-S3.
With the ESP32-S3 I found, I would have the great amount of 16mb flash, 8mb RAM, and 240mhz (While arduino only had 2mb flash, a few kb of RAM and 16mhz).
I bought the ESP32 and tested it with the screen, and this is what I realised... in the next journal.

![image.png](https://cdn.hackclub.com/019e70e4-26a3-7585-b1a7-4c293bf3d547/image.png)

References: 
LovyanGFX Graphics Lib Docs I created: [docs](/user-attachments/blobs/proxy/eyJfcmFpbHMiOnsiZGF0YSI6MTI4MTgwLCJwdXIiOiJibG9iX2lkIn19--34296b57b84e30e53a206c20e1219ba0597a7385/docs.txt)
TFT Screen Pins: [pins and instructions](/user-attachments/blobs/proxy/eyJfcmFpbHMiOnsiZGF0YSI6MTI4MTgxLCJwdXIiOiJibG9iX2lkIn19--d9cf887b7b4b8194627fff10a0912c5fef83ded2/pins%20and%20instructions.txt)
TFT Screen datasheet: [3.5inch_SPI_Module_MSP3525_MSP3526_User_Manual_EN](/user-attachments/blobs/proxy/eyJfcmFpbHMiOnsiZGF0YSI6MTI4MjAwLCJwdXIiOiJibG9iX2lkIn19--e6029fc0dd42921700cebb05dfc5681f751da224/3.5inch_SPI_Module_MSP3525_MSP3526_User_Manual_EN.pdf)
ESP32 S3 datasheet: [esp32-s3_datasheet_en](/user-attachments/blobs/proxy/eyJfcmFpbHMiOnsiZGF0YSI6MTI4MTgzLCJwdXIiOiJibG9iX2lkIn19--8f73b24e2fe7c50b8681c89f8a55815ece124108/esp32-s3_datasheet_en.pdf)
KiCad tutorial: https://www.youtube.com/watch?v=mmhareh99P8
Blender tutorial: https://www.youtube.com/watch?v=peSv5IT5Ve4
LovyanGFX tutorial: https://www.youtube.com/watch?v=IPCvQ4o_WP8

# 2026-05-28: 2026-05-18: 1/26/2026 - Design Edited and More Powering Tests

**Total time spent: 2.5h**

**Total time spent: 2.5h**

Time spent: 2.5h  

Today I've made some changes on the design. Added a USB port, added the esp32 on the board, modified the hinges to be more tight to the hinge "connector).

I also tested the light sleep and tft sleep functions, reset using software and googled about power consumption. Everything now is schematized.

I made a chart about the powering:
![imagem_2026-01-26_164341373](https://blueprint.hackclub.com/user-attachments/blobs/proxy/eyJfcmFpbHMiOnsiZGF0YSI6OTIyMDAsInB1ciI6ImJsb2JfaWQifX0=--f1d5ed56aafea86e646d89e5facfebdd9489e9de/imagem_2026-01-26_164341373.png)
Powering looks decent and so the battery duration.

# 2026-05-28: 2026-05-18: 2/7/2026 - OS Design

**Total time spent: 1.0h**

**Total time spent: 1.0h**

Time spent: 1.0h  

In an hour, I made the OS design and pages in paint.
I want the design to be a little Linux Like, kinda cozy, and easy to use. It will work with an arrow system. With WASD you control where your "cursor" is, like game consoles.

Starting:
![image](https://blueprint.hackclub.com/user-attachments/blobs/proxy/eyJfcmFpbHMiOnsiZGF0YSI6OTk2MjksInB1ciI6ImJsb2JfaWQifX0=--7888eb3ec57eb279e7b22be18cbe743046925937/image.png)

Login:
![image](https://blueprint.hackclub.com/user-attachments/blobs/proxy/eyJfcmFpbHMiOnsiZGF0YSI6OTk2MzAsInB1ciI6ImJsb2JfaWQifX0=--756ee2fe644b34610bcb12731faa80a48224071c/image.png)

Home Page:
![image](https://blueprint.hackclub.com/user-attachments/blobs/proxy/eyJfcmFpbHMiOnsiZGF0YSI6OTk2MjgsInB1ciI6ImJsb2JfaWQifX0=--49b55430845738d0b215d3726487313dc5c966ac/image.png)

# 2026-05-28: 2026-05-18: 2/10/2026 - Stickers and Powering Issues

**Total time spent: 0.3h**

**Total time spent: 0.3h**

Time spent: 0.3h

Today I've finally recieved my stickers, and they are so cute!

Loved the processor one!!

![20260210_185206](https://blueprint.hackclub.com/user-attachments/blobs/proxy/eyJfcmFpbHMiOnsiZGF0YSI6MTAxODk4LCJwdXIiOiJibG9iX2lkIn19--748e92104835db2ac6bed7cd86872b72a087455a/20260210_185206.jpg)

![20260210_185324](https://blueprint.hackclub.com/user-attachments/blobs/proxy/eyJfcmFpbHMiOnsiZGF0YSI6MTAxODk5LCJwdXIiOiJibG9iX2lkIn19--306cd2bc400b15e44527433316792f43cfddd4b1/20260210_185324.jpg)

To frustrate a little bit the day, though, the battery holders didn't work. It just doesn't charge and neither transfers the electricity to the esp32.
To solve that, I will use a power bank that must have 2 USBS (one for charging, and one for output) and a powering on/off button.
I will use a power bank module to prioritize safety, reduce development risk, and keep the project scope focused on PCB design, firmware, and system integration. The power bank provides built-in protection and regulated output.

# 2026-05-28: 2026-05-18: 2/27/2026 - System's Kernel and Tests

**Total time spent: 12.0h**

**Total time spent: 12.0h**

Time spent: 12.0h

During 4 days (From 4 days ago to today), I've been developing the OS Kernel.
I've spent great 12 hours developing the kernel main functions and folder structure.

The kernel now starts screen, starts sd module, loads boot configs, creates screen frame, loads general configs, synchronize time using wifi to obtain real life time and set it to the RTC and then turns off wifi to save energy, loads user, starts first lua vm (OS Graphic Interface).

OS functions:
Display_ClearScreen, Display_UpdateScreen, OS_GetFreeMemory, OS_GetMemorySize, OS_GetFreeHeapMemory, OS_GetHeapMemorySize, OS_GetCPUTemperature, OS_GetTime:toString("%d/%m/%Y - %H:%M:%S"), OS_GetLuaVMHeapMemoryUsage, OS_GetUserUsername, OS_GetUserPassword

Furthermore, I've made the Sprite exported structure and exported lua variable type and so with Fonts.
This is a simple example lua code of how the system is working:
```
local user_login_bg = Sprite("/sys/img/user_login_bg.png", 480, 320)
local gothambold_font = Font("/sys/fonts/Gotham Bold.bin")

-- FPS
local fps = 0
local frameCount = 0
local lastTime = os.clock()

function loop()
   while true do
      local currentTime = os.clock()
      frameCount = frameCount + 1

      -- Atualiza FPS a cada 1 segundo
      if currentTime - lastTime >= 1 then
         collectgarbage("collect")
         fps = frameCount
         frameCount = 0
         lastTime = currentTime
         --print("FPS:", fps)
         --print("Temperature: ", OS_GetCPUTemperature())
         --print("Free Memory: ", OS_GetFreeHeapMemory())
         --print("Memory Size: ", OS_GetMemorySize())
         --print("RTC: ", OS_GetTime():toString("%d/%m/%Y - %H:%M:%S"))
         --print("Lua Heap Memory Usage: ", OS_GetLuaVMHeapMemoryUsage())
      end

      Display_ClearScreen()

      user_login_bg:draw(0, 0)
      gothambold_font:drawString(OS_GetUserUsername(), 148, 150, 20, 255, 255, 255)

      Display_UpdateScreen()
   end
end

function start()
   loop()
end

start()
endProgram()

```

Images are always in .png, supported transparency, and fonts are always in .bin.
The fonts are converted from ttf to binary files using a python script avaliable on github.

The most difficult part was the Lua VM memory management: In the beginning, there was a big memory leak in LUA VMs. The solution was to call every second collectgarbage("collect").
And also: the luaHeapUsed variable was being miscounted, causing a memory leak.

Here is the OS booting and start page:
![WhatsApp Image 2026-02-27 at 21.43.36(1)](https://blueprint.hackclub.com/user-attachments/blobs/proxy/eyJfcmFpbHMiOnsiZGF0YSI6MTEyNTcyLCJwdXIiOiJibG9iX2lkIn19--fad7c37d7f5d2ae81a223833f3933576445f9c32/WhatsApp%20Image%202026-02-27%20at%2021.43.36(1).jpeg)
![WhatsApp Image 2026-02-27 at 21.43.36](https://blueprint.hackclub.com/user-attachments/blobs/proxy/eyJfcmFpbHMiOnsiZGF0YSI6MTEyNTczLCJwdXIiOiJibG9iX2lkIn19--fc565008f973b24c9f828267c535f0b0d1358455/WhatsApp%20Image%202026-02-27%20at%2021.43.36.jpeg)

# 2026-05-18: 5/18/2026 - Transference from Blueprint to Forge program

**Total time spent: 0.1h_**

_Time spent: 0.1h_  

After 2 months of waiting, I decided to transfer Blueprint to Forge.

Now, Im going to wait anxiously to the final review.

hahahahahahahahahahahahahahaha

![image](https://blueprint.hackclub.com/user-attachments/blobs/proxy/eyJfcmFpbHMiOnsiZGF0YSI6MTYwNjU1LCJwdXIiOiJibG9iX2lkIn19--524e89be46406af7e8b07ccf29cad5bd0bef55a9/image.png)

# 2026-05-18: 2/27/2026 - System's Kernel and Tests

**Total time spent: 12.0h_**

_Time spent: 12.0h_  

During 4 days (From 4 days ago to today), I've been developing the OS Kernel.
I've spent great 12 hours developing the kernel main functions and folder structure.

The kernel now starts screen, starts sd module, loads boot configs, creates screen frame, loads general configs, synchronize time using wifi to obtain real life time and set it to the RTC and then turns off wifi to save energy, loads user, starts first lua vm (OS Graphic Interface).

OS functions:
Display_ClearScreen, Display_UpdateScreen, OS_GetFreeMemory, OS_GetMemorySize, OS_GetFreeHeapMemory, OS_GetHeapMemorySize, OS_GetCPUTemperature, OS_GetTime:toString("%d/%m/%Y - %H:%M:%S"), OS_GetLuaVMHeapMemoryUsage, OS_GetUserUsername, OS_GetUserPassword

Furthermore, I've made the Sprite exported structure and exported lua variable type and so with Fonts.
This is a simple example lua code of how the system is working:
```
local user_login_bg = Sprite("/sys/img/user_login_bg.png", 480, 320)
local gothambold_font = Font("/sys/fonts/Gotham Bold.bin")

-- FPS
local fps = 0
local frameCount = 0
local lastTime = os.clock()

function loop()
   while true do
      local currentTime = os.clock()
      frameCount = frameCount + 1

      -- Atualiza FPS a cada 1 segundo
      if currentTime - lastTime >= 1 then
         collectgarbage("collect")
         fps = frameCount
         frameCount = 0
         lastTime = currentTime
         --print("FPS:", fps)
         --print("Temperature: ", OS_GetCPUTemperature())
         --print("Free Memory: ", OS_GetFreeHeapMemory())
         --print("Memory Size: ", OS_GetMemorySize())
         --print("RTC: ", OS_GetTime():toString("%d/%m/%Y - %H:%M:%S"))
         --print("Lua Heap Memory Usage: ", OS_GetLuaVMHeapMemoryUsage())
      end

      Display_ClearScreen()

      user_login_bg:draw(0, 0)
      gothambold_font:drawString(OS_GetUserUsername(), 148, 150, 20, 255, 255, 255)

      Display_UpdateScreen()
   end
end

function start()
   loop()
end

start()
endProgram()

```

Images are always in .png, supported transparency, and fonts are always in .bin.
The fonts are converted from ttf to binary files using a python script avaliable on github.

The most difficult part was the Lua VM memory management: In the beginning, there was a big memory leak in LUA VMs. The solution was to call every second collectgarbage("collect").
And also: the luaHeapUsed variable was being miscounted, causing a memory leak.

Here is the OS booting and start page:
![WhatsApp Image 2026-02-27 at 21.43.36(1)](https://blueprint.hackclub.com/user-attachments/blobs/proxy/eyJfcmFpbHMiOnsiZGF0YSI6MTEyNTcyLCJwdXIiOiJibG9iX2lkIn19--fad7c37d7f5d2ae81a223833f3933576445f9c32/WhatsApp%20Image%202026-02-27%20at%2021.43.36(1).jpeg)
![WhatsApp Image 2026-02-27 at 21.43.36](https://blueprint.hackclub.com/user-attachments/blobs/proxy/eyJfcmFpbHMiOnsiZGF0YSI6MTEyNTczLCJwdXIiOiJibG9iX2lkIn19--fc565008f973b24c9f828267c535f0b0d1358455/WhatsApp%20Image%202026-02-27%20at%2021.43.36.jpeg)

# 2026-05-18: 2/12/2026 - Powering

**Total time spent: 0.3h_**

_Time spent: 0.3h_  

Today I finally found the perfect powering system to this project: Futurizta Power Bank 5000mah Mini.

This power bank has 5000mah (Battery would last so longer than if I've bought a AAA battery), so it would last 27h. And with the WIFI on: 14.5h.

In addition, it is so tiny being only 7.5x3.5x2cm, fitting perfectly inside the computer.

This project is gonna be really fat. The lower part will be 4cm tall.

![Screenshot_20260212_091511_Opera GX](https://blueprint.hackclub.com/user-attachments/blobs/proxy/eyJfcmFpbHMiOnsiZGF0YSI6MTAyODc3LCJwdXIiOiJibG9iX2lkIn19--759f881e9137a489ce3d2ade971acd37526cf9fd/Screenshot_20260212_091511_Opera%20GX.jpg)

So, the powering chart is like this now:
![image](https://blueprint.hackclub.com/user-attachments/blobs/proxy/eyJfcmFpbHMiOnsiZGF0YSI6MTI1MTExLCJwdXIiOiJibG9iX2lkIn19--6597fe36b5fe80526b3fae7c83df5907cb1d0270/image.png)

# 2026-05-18: 2/10/2026 - Stickers and Powering Issues

**Total time spent: 0.3h_**

_Time spent: 0.3h_  

Today I've finally recieved my stickers, and they are so cute!

Loved the processor one!!

![20260210_185206](https://blueprint.hackclub.com/user-attachments/blobs/proxy/eyJfcmFpbHMiOnsiZGF0YSI6MTAxODk4LCJwdXIiOiJibG9iX2lkIn19--748e92104835db2ac6bed7cd86872b72a087455a/20260210_185206.jpg)

![20260210_185324](https://blueprint.hackclub.com/user-attachments/blobs/proxy/eyJfcmFpbHMiOnsiZGF0YSI6MTAxODk5LCJwdXIiOiJibG9iX2lkIn19--306cd2bc400b15e44527433316792f43cfddd4b1/20260210_185324.jpg)

To frustrate a little bit the day, though, the battery holders didn't work. It just doesn't charge and neither transfers the electricity to the esp32.
To solve that, I will use a power bank that must have 2 USBS (one for charging, and one for output) and a powering on/off button.
I will use a power bank module to prioritize safety, reduce development risk, and keep the project scope focused on PCB design, firmware, and system integration. The power bank provides built-in protection and regulated output.

# 2026-05-18: 2/7/2026 - OS Design

**Total time spent: 1.0h_**

_Time spent: 1.0h_  

In an hour, I made the OS design and pages.
I want the design to be a little Linux Like, kinda cozy, and easy to use. It will work with an arrow system. With WASD you control where your "cursor" is, like game consoles.

Starting:
![image](https://blueprint.hackclub.com/user-attachments/blobs/proxy/eyJfcmFpbHMiOnsiZGF0YSI6OTk2MjksInB1ciI6ImJsb2JfaWQifX0=--7888eb3ec57eb279e7b22be18cbe743046925937/image.png)

Login:
![image](https://blueprint.hackclub.com/user-attachments/blobs/proxy/eyJfcmFpbHMiOnsiZGF0YSI6OTk2MzAsInB1ciI6ImJsb2JfaWQifX0=--756ee2fe644b34610bcb12731faa80a48224071c/image.png)

Home Page:
![image](https://blueprint.hackclub.com/user-attachments/blobs/proxy/eyJfcmFpbHMiOnsiZGF0YSI6OTk2MjgsInB1ciI6ImJsb2JfaWQifX0=--49b55430845738d0b215d3726487313dc5c966ac/image.png)

# 2026-05-18: 2/6/2026 - README, Github, Name and Notes for the reviewer

**Total time spent: 1.0h_**

_Time spent: 1.0h_  

During these 2 days, I've been writing the README and configuring the Github Repo.

The README is already avaliable on Github.

I also changed the project's name, to be more, shocking...

Added some notes for the reviewer about why 2 computers and the lack of a correct .step (Current is 4gb).

![image](https://blueprint.hackclub.com/user-attachments/blobs/proxy/eyJfcmFpbHMiOnsiZGF0YSI6OTkzMDksInB1ciI6ImJsb2JfaWQifX0=--4e33bb4490a814440b3904b9e3803141445c06ff/image.png)

Furthermore, I made the wiring diagram for parts out of the pcb.
![wiring diagram](https://blueprint.hackclub.com/user-attachments/blobs/proxy/eyJfcmFpbHMiOnsiZGF0YSI6OTkzMTMsInB1ciI6ImJsb2JfaWQifX0=--f7bced60d88d1517b670a2406941892d914df3e7/wiring%20diagram.png)

# 2026-05-18: 2/3/2026 - Case Fixes and More PCB Quotings

**Total time spent: 2.0h_**

_Time spent: 2.0h_  

During these days, I've added screw holes to my case, and discovered that the JLCPCB don't have the part I need (Pinheader Pitch 2.54mm 8mm height), so I'm going to buy this part from AliExpress, and solder it by myself with a borrowed Solding Station from the theater lights guy.

I also converted the .blend into a .step file. It had a some of errors and convertion isn't precise, but ok. It is 4gb...

In addition, to the 3D prints, I found a guy that prints inside Blueprint (I forgot the program name). And he just ask for the shipping price and fillament.

![imagem_2026-02-03_190743825](https://blueprint.hackclub.com/user-attachments/blobs/proxy/eyJfcmFpbHMiOnsiZGF0YSI6OTc4MDQsInB1ciI6ImJsb2JfaWQifX0=--3cc36f9502937673fe28ba041904c46183192d18/imagem_2026-02-03_190743825.png)

![imagem_2026-02-03_190820493](https://blueprint.hackclub.com/user-attachments/blobs/proxy/eyJfcmFpbHMiOnsiZGF0YSI6OTc4MDUsInB1ciI6ImJsb2JfaWQifX0=--5e55f74c9a0d1fba02d5b3f738c5a6e715640430/imagem_2026-02-03_190820493.png)

# 2026-05-18: 1/31/2026 - First Firmware steps and OS

**Total time spent: 6.0h_**

_Time spent: 6.0h_  

Today, I've researched some ways to run the Firmware, multitasking and apps.

So, the problem was about the apps: How the system would run multiple apps with open software and that wouldn't depend on just the base firmware and save states, I mean, if I put every app inside the firmware (.ino), the apps wouldn't be updateable easily. Everything would be linked in a way that: If a software breaks, all the system breaks, and I would have to compile the ENTIRE SKETCH again and again.

To solve it, I decided to run a Lua interpreter within Esp32.

This will allow me to create: The bios, firmware, OS and apps runner separately from the apps themselves.

The firmware and OS will be created apart in a .ino sketch, and the apps will have their own folder inside the SD Card and their own folder structure. Generally containing fonts, images, config json, .lua scripting files.

The OS job will only be read the lua script and interpret it.

There will be a ton of external functions which could be used to create the apps, such as: File functions, Drawing, Math, OS management (Configs, restart, shutdown, close app, etc). The definitions and instructions controlled and set by the C++ sketch (Firmware).

This is the base of the firmware. So far it has only the screen and sd init, interpreter starter and one external function: endProgram() to end the VM.

![imagem_2026-01-31_182833809](https://blueprint.hackclub.com/user-attachments/blobs/proxy/eyJfcmFpbHMiOnsiZGF0YSI6OTU4NjksInB1ciI6ImJsb2JfaWQifX0=--957d23f0206282a48f3ed2b585673e3b99ef78ae/imagem_2026-01-31_182833809.png)

# 2026-05-18: 1/30/2026 - Tracked Prices

**Total time spent: 2.0h_**

Time spent: 2.0h

During these 2 days of work, I've made some work on the PCB -fixed the tracks that were not straight[tortas] (idk how to say it in english) and revised all the tracks- and routed where I should manufacture them.

I started calling a lot of local factories, but they were so expensive.

So, I will manufacture everything with JLCPCB. I've added the general board to my cart, but 1 component for the keyboard were in inventory shortage, so I pre-ordered by 5ish dollars the Pin Header I need.

The problem when quoting on JLCPCB was that my BOM was really messed up, and I had to redo it using the JLCPCB pattern. This took me a lot of time.
And the other problem was that I had to search for the components, and it took so much time. The time consuming one were the pin headers and pin sockets. Some were not avaliable, so I had to make a pre-order.

Everything will cost approximately 700 reais (5 general boards and 5 keyboards. 2 assembled).

Plus the 3D print for the case will cost ~18 dollars (Two PCs) (JayG from HackClub Printing Stuff).

The screen: ~140 reais (paid by me)
The ESP32: ~90 reais (paid by me)

![imagem_2026-01-30_211424497](https://blueprint.hackclub.com/user-attachments/blobs/proxy/eyJfcmFpbHMiOnsiZGF0YSI6OTU0NTEsInB1ciI6ImJsb2JfaWQifX0=--8c480420b0e525c17d8782693c50ae5fa6362665/imagem_2026-01-30_211424497.png)

# 2026-05-18: 1/26/2026 - Design Edited and More Powering Tests

**Total time spent: 2.5h_**

_Time spent: 2.5h_  

Today I've made some changes on the design. Added a USB port, added the esp32 on the board, modified the hinges to be more tight to the hinge "connector).

I also tested the light sleep and tft sleep functions, reset using software and googled about power consumption. Everything now is schematized.

I made a chart about the powering:
![imagem_2026-01-26_164341373](https://blueprint.hackclub.com/user-attachments/blobs/proxy/eyJfcmFpbHMiOnsiZGF0YSI6OTIyMDAsInB1ciI6ImJsb2JfaWQifX0=--f1d5ed56aafea86e646d89e5facfebdd9489e9de/imagem_2026-01-26_164341373.png)
Powering looks decent and so the battery duration.

# 2026-05-18: 2/27/2026 - System's Kernel and Tests

**Total time spent: 12.0h_**

_Time spent: 12.0h_  

During 4 days (From 4 days ago to today), I've been developing the OS Kernel.
I've spent great 12 hours developing the kernel main functions and folder structure.

The kernel now starts screen, starts sd module, loads boot configs, creates screen frame, loads general configs, synchronize time using wifi to obtain real life time and set it to the RTC and then turns off wifi to save energy, loads user, starts first lua vm (OS Graphic Interface).

OS functions:
Display_ClearScreen, Display_UpdateScreen, OS_GetFreeMemory, OS_GetMemorySize, OS_GetFreeHeapMemory, OS_GetHeapMemorySize, OS_GetCPUTemperature, OS_GetTime:toString("%d/%m/%Y - %H:%M:%S"), OS_GetLuaVMHeapMemoryUsage, OS_GetUserUsername, OS_GetUserPassword

Furthermore, I've made the Sprite exported structure and exported lua variable type and so with Fonts.
This is a simple example lua code of how the system is working:
```
local user_login_bg = Sprite("/sys/img/user_login_bg.png", 480, 320)
local gothambold_font = Font("/sys/fonts/Gotham Bold.bin")

-- FPS
local fps = 0
local frameCount = 0
local lastTime = os.clock()

function loop()
   while true do
      local currentTime = os.clock()
      frameCount = frameCount + 1

      -- Atualiza FPS a cada 1 segundo
      if currentTime - lastTime >= 1 then
         collectgarbage("collect")
         fps = frameCount
         frameCount = 0
         lastTime = currentTime
         --print("FPS:", fps)
         --print("Temperature: ", OS_GetCPUTemperature())
         --print("Free Memory: ", OS_GetFreeHeapMemory())
         --print("Memory Size: ", OS_GetMemorySize())
         --print("RTC: ", OS_GetTime():toString("%d/%m/%Y - %H:%M:%S"))
         --print("Lua Heap Memory Usage: ", OS_GetLuaVMHeapMemoryUsage())
      end

      Display_ClearScreen()

      user_login_bg:draw(0, 0)
      gothambold_font:drawString(OS_GetUserUsername(), 148, 150, 20, 255, 255, 255)

      Display_UpdateScreen()
   end
end

function start()
   loop()
end

start()
endProgram()

```

Images are always in .png, supported transparency, and fonts are always in .bin.
The fonts are converted from ttf to binary files using a python script avaliable on github.

The most difficult part was the Lua VM memory management: In the beginning, there was a big memory leak in LUA VMs. The solution was to call every second collectgarbage("collect").
And also: the luaHeapUsed variable was being miscounted, causing a memory leak.

Here is the OS booting and start page:
![WhatsApp Image 2026-02-27 at 21.43.36(1)](https://blueprint.hackclub.com/user-attachments/blobs/proxy/eyJfcmFpbHMiOnsiZGF0YSI6MTEyNTcyLCJwdXIiOiJibG9iX2lkIn19--fad7c37d7f5d2ae81a223833f3933576445f9c32/WhatsApp%20Image%202026-02-27%20at%2021.43.36(1).jpeg)
![WhatsApp Image 2026-02-27 at 21.43.36](https://blueprint.hackclub.com/user-attachments/blobs/proxy/eyJfcmFpbHMiOnsiZGF0YSI6MTEyNTczLCJwdXIiOiJibG9iX2lkIn19--fc565008f973b24c9f828267c535f0b0d1358455/WhatsApp%20Image%202026-02-27%20at%2021.43.36.jpeg)

# 2026-05-18: 1/30/2026 - Tracked Prices

**Total time spent: 5.0h_**

_Time spent: 5.0h_  

During these 2 days of work, I've made some work on the PCB -fixed the tracks that were not straight[tortas] (idk how to say it in english) and revised all the tracks- and routed where I should manufacture them.

I started calling a lot of local factories, but they were so expensive.

So, I will manufacture everything with JLCPCB. I've added the general board to my cart, but 1 component for the keyboard were in inventory shortage, so I pre-ordered by 5ish dollars the Pin Header I need.

The problem when quoting on JLCPCB was that my BOM was really messed up, and I had to redo it using the JLCPCB pattern. This took me a lot of time.
And the other problem was that I had to search for the components, and it took so much time. The time consuming one were the pin headers and pin sockets. Some were not avaliable, so I had to make a pre-order.

Everything will cost approximately 700 reais (5 general boards and 5 keyboards. 2 assembled).

Plus the 3D print for the case will cost ~18 dollars (Two PCs) (JayG from HackClub Printing Stuff).

The screen: ~140 reais (paid by me)
The ESP32: ~90 reais (paid by me)

![imagem_2026-01-30_211424497](https://blueprint.hackclub.com/user-attachments/blobs/proxy/eyJfcmFpbHMiOnsiZGF0YSI6OTU0NTEsInB1ciI6ImJsb2JfaWQifX0=--8c480420b0e525c17d8782693c50ae5fa6362665/imagem_2026-01-30_211424497.png)


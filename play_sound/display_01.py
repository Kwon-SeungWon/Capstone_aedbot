from PIL import Image
from Xlib import display


def display_image(image_path):
    # Open the image using PIL
    image = Image.open(image_path)

    # Get the screen size
    screen = display.Display().screen().root
    width = screen.get_geometry().width
    height = screen.get_geometry().height

    # Resize the image to fit the screen
    image = image.resize((width, height))

    # Create a blank window in full screen
    window = display.Display().create_window(
        screen,
        0,
        0,
        width,
        height,
        0,
        screen.root_depth,
        display.CopyFromParent,
        visual=screen.root_visual,
        fullscreen=True,
    )

    # Map the window to make it visible
    window.map()

    # Create a graphics context
    gc = window.create_gc()

    # Load the image data
    data = image.tobytes()

    # Create an XImage with the image data
    ximage = display.Display().create_image(
        screen, screen.root_visual.depth, display.ZPixmap, 0, data, width, height
    )

    # Put the image on the window
    window.put_image(gc, 0, 0, ximage)

    # Flush the changes to the display
    display.Display().flush()

    # Wait for a key press or window close event
    running = True
    while running:
        event = display.Display().next_event()
        if event.type == display.KeyPress or event.type == display.ButtonPress:
            running = False


# Path to the image you want to display
image_path = "SampleJPGImage_50kbmb.jpg"

# Call the function to display the image
display_image(image_path)

# Pica (3D Physics Engine)

This project is a physics library extension for citro3D, providing a lightweight alternative for physics-based simulation. Currently undergoing bug-bashing session to fix up issues. 

The physics engine is a port from [qu3e](https://github.com/RandyGaul/qu3e), written by Randy Gual. Permission was granted for porting the code over to the Nintendo 3DS family systems.

citro3d is a library that provides an easy to use stateful interface to the PICA200 GPU of the Nintendo 3DS. It tries to expose hardware functionality in the way that is most natural and convenient to the GPU and the user, therefore deviating from openGL. citro3d is written by fincs, with contributions from Cruel`, mtheall, and tommai78101.

Yes, citro3d is spelled with a lowercase C.

It is written in C programming language.

### Why does the physics library depend on citro3d?

Two reasons:

1. It relies heavily on the math functions integrated into citro3d.
2. Ease of setting things up.

# Setup

Pica (and citro3d altogether) can be built and installed using the following command (should not expect any issues here):

    make install

# Demonstration Code

Please check the `/demo` directory or in the repository above. Currently under construction.

# License

  This software is provided 'as-is', without any express or implied
  warranty.  In no event will the authors be held liable for any
  damages arising from the use of this software.

  Permission is granted to anyone to use this software for any
  purpose, including commercial applications, and to alter it and
  redistribute it freely, subject to the following restrictions:

  1. The origin of this software must not be misrepresented; you
     must not claim that you wrote the original software. If you use
     this software in a product, an acknowledgment in the product
     documentation would be appreciated but is not required.
  2. Altered source versions must be plainly marked as such, and
     must not be misrepresented as being the original software.
  3. This notice may not be removed or altered from any source
     distribution.

# Additional Credits

Thanks to LinkBlaBla for the project name, Pica. It is inspired by the graphics chipset on the Nintendo 3DS, named PICA200.
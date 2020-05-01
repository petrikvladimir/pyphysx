
import sys

sys.path.append('lib')
print(sys.path)

import unittest
from pyphysx import Material, Physics

class MyTestCase(unittest.TestCase):
    def test_something(self):
        self.assertEqual(True, True)

    def test_material(self):
        physics = Physics()
        mat = Material()
        self.assertEqual(mat.get_static_friction(), 0.)


if __name__ == '__main__':
    unittest.main()

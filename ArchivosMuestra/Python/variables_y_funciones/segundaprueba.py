#!/usr/bin/env python3
"""
Esta librería contiene definiciones
para clases de Pokemon y Ataques
"""
class TestClass():
  x = 8
  y = "hola"
  def test_func(self):
    return self.x
  
class Ataque():
  """
  Representa un ataque
  """
  def __init__(self, nombre:str, poder:int)->'Ataque':
    """
    Inicialización del ataque
    Args:
      :param nombre: (str) Nombre del ataque
      :param nivel: (int) Poder del ataque
    """
    self.nombre = nombre
    self.poder = poder
    
class Pokemon():
  """
  Representa un pokémon
  """
  def __init__(self, nombre:str, nivel:int):
    """
    Inicialización del pokemon
    Args:
      nombre (str): Nombre del pokemon
      nivel (int): Nivel del pokemon
    """
    self.nombre = nombre
    self.nivel = nivel
    self.ataques = []
  def agregar_ataque(self, ataque:Ataque)->bool:
    """
    Agrega un ataque al pokémon
    Args:
      ataque (Ataque): Ataque a agregar
    """
    self.ataques.append(ataque)
    print("El pokemon {} aprendió {}".format(self.nombre, 
                                             ataque.nombre))
    return True


if __name__ == "__main__":
  pokemon = Pokemon("Pikachu", 5)
  ataque = Ataque("Trueno", 120)
  pokemon.agregar_ataque(ataque)
else:
  print("Se importó el script " + __name__)


"""
testClass = TestClass()
a = testClass.test_func()
print(testClass.test_func())
print(a)
"""
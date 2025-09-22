x = 5
y = float(6.2)
z = {1, 2, 3, 4}

#print(y)

def funcion_prueba(param1:str='a', param2:float=8.0, param3:int=1):
  a = param1
  b = param2
  c = param3
  print("Se llamó a la función")
  print(c)
  return
def segunda_funcion(*args):
  #Formas de crear una string
  #Formato
  print("Se recibieron {} argumentos.".format(len(args)))
  #Concatenando
  print("Se recibieron " + str(len(args)) + " argumentos.")
  print("""Esta es
        una string
        con saltos de línea""")
  """
  El bloque siguiente
  suma los parámetros 
  de la función
  """

  suma = 0
  for i in args:
    suma = suma + i
  return suma

  
"""
funcion_prueba(param3=5)
suma = segunda_funcion(8, 5, 10,15, 29)
print(suma)
"""

def tercera_funcion():
  #Lista
  lista = ['a', 23, "Hola"]
  print(lista[0])
  lista.append(23)
  lista.remove("Hola")
  print(lista)
  #Conjunto
  conjunto = {1,2,3,3,4,4}
  print(conjunto)
  #Diccionario
  dicc = {"Edad":20, "Nombre":"Juan", "Carrera":"Mecánica"}
  print(dicc["Carrera"])
  #Tupla
  tupla = (1,4,6,'a',"Hola")
  a, b, c, d, e = tupla
  print(e)
  _, _, h, _, _ = tupla
  print(h)
  pass

tercera_funcion()

#!/bin/bash
# ==================================================
# Script de ejemplo para mostrar funcionalidades básicas de bash
# ==================================================

echo  ---- 1. Imprimir en pantalla ----
echo "Hola! Este es un ejemplo de script en bash"
echo "------------------------------------------"

echo  ---- 2. Variables ----
TEXTO="Hola"
NUMERO=5
echo "$TEXTO a todos! $NUMERO"

echo  ---- 3. Operaciones aritméticas ----
SUMA=$((NUMERO + 10))
echo "El número $NUMERO + 10 es $SUMA"

echo  --- 4. Leer entrada del usuario ----
read -p "Escribe tu edad: " EDAD
echo "Tienes $EDAD años"

echo  ---- 5. Condicionales ----
if [ $EDAD -ge 18 ]; then
    echo "Eres mayor de edad"
else
    echo "Eres menor de edad"
fi

echo  ---- 6. Ciclo for ----
echo "Contando del 1 al 5 con un ciclo for:"
for i in {1..5}; do
    echo "Número: $i"
done

echo "Ciclo for con un arreglo manual:"
    for color in 1 3 8 5 -2; do
        echo "Color: $color"
    done

echo  ---- 7. Ciclo while ----
# Archivos: -e (exists), -f (regular file), -d (directory), -r (readable), -w (writable), -x (executable)
# Compraciones: -eq (equal to), -ne (not equal to), -gt (greater than), -ge (greater than or equal to), -lt (less than), -le (less than or equal to)
#Comparación de Strings: == (equal to), != (not equal to), < (lexicographically less than), > (lexicographically greater than), -z (string is empty), -n (string is not empty).
echo "Contando del 5 al 1 con un ciclo while:"
x=5
while [ $x -gt 0 ]; do
    echo "Número: $x"
    ((x--))
done

echo  ---- 8. Funciones ----
saludar() {
    echo "Mensaje para ti:  $1"
}
saludar "Hola"

echo  ---- 9. Manejo de archivos y directorios ----
#Verifica si el directorio existe
if test -d "carpeta_ejemplo"; then
    echo "La carpeta 'carpeta_ejemplo' ya existe"
else
    mkdir carpeta_ejemplo
    echo "Este es un archivo de prueba" > carpeta_ejemplo/archivo.txt
    cp carpeta_ejemplo/archivo.txt carpeta_ejemplo/archivo_copia.txt
    mv carpeta_ejemplo/archivo_copia.txt carpeta_ejemplo/renombrado.txt
    echo "Se creó la carpeta 'carpeta_ejemplo' con un archivo y se copió/renombró"
fi
    


echo  ---- 10. Listar archivos ----
echo "Archivos en la carpeta_ejemplo:"
ls carpeta_ejemplo

echo  ---- 11. Parámetros del script ----
# Para agregar parámetros usar:
# source EjemploBash.sh a=1 b=2 c=3
echo "Parámetros recibidos: $@"
echo "Número de parámetros: $#"

echo  ---- 12. Mostrar documentación de un comando ----
echo "Especificaciones del comando 'cat':"
cat --help
echo  ---- 13. Finalización ----
echo "------------------------------------------"
echo "Script finalizado correctamente ✅"

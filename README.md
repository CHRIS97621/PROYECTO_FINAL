## Crear un git
### Para agregar y hacer seguimiento de arvhivos
```bash
git add .
```
### Para hacer commit
```bash
git commit -m "Primer commit"
```

### Estado de mis archivos


```bash
git status
```

### Historial de cambios


```bash
git log
```

### cambiar el nombre de un branch


```bash
git branch -M main
```


### Instalar extensiones
ROS
URDF


### URDF

#### CONVERTIR DE xacro a urdf
```bash
xacro agro.urdf.xacro >> agro.urdf
# xacro path_file_xacro.xacro >> path_destinacion.urdf
```
```bash
colcon build --symlink-install
```

### trabajo

#### 1.1  Sacar medidas de solidworks, del chassis y ruedas como figuras geometricas simples
#### 2.2  Llevar los centro de referencia al centro geometris de cada parte
#### 2.3  Descar los archivos en .stl, chassis, ruedas y acoples
#### 2.4  Por ahora no se considerar los acoples
#### 2.5  Crear el xacro con figuras geometricas simples y pobrar en rviz
#### 2.6  Reemplzar las figuras geometricas simples con mesh
#### 2.7  Probar con una rueda  reutilziar con xacros las demás ruedas
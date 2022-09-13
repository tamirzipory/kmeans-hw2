from setuptools import setup, Extension
module = Extension("spkmean",
                  sources=[
                    'spkmeans.c',
                    'spkmeansmodule.c'
                  ])

setup(
    name= "spkmean",
    version= "1.0",
    description= "kmeans c function",
    ext_modules= [
	    Extension(
		    'spkmean', 
		    ['spkmeansmodule.c', 'spkmeans.c'],
	    ),
	]
)

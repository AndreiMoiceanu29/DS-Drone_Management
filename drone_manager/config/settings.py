"""Project Settings Module."""
from pathlib import Path


BASE_DIR = Path(__file__).resolve().parent.parent

SECRET_KEY = 'Z7$%L!9.<eVNa_]E&]s$&`P=V3n~5F+^5(Hb6,&f'

JWT_ALGORITHM = 'HS256'
JWT_EXP_DELTA_SECONDS = 60 * 60


DATABASES = {
    'mongodb': {
        'USER': 'test',
        'PASSWORD': 'test',
        'HOST': '172.20.0.5',
        'PORT': '27017'
    }
}
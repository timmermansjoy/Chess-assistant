FROM python:3.9.2-alpine3.12


COPY . /Chess
WORKDIR /Chess
RUN apk add --update zsh git
RUN sh -c "$(wget https://raw.github.com/robbyrussell/oh-my-zsh/master/tools/install.sh -O -)"
RUN  pip3 install -r requirements.txt
ENTRYPOINT /bin/zsh

# -- to run python code instantly
# RUN pip install docker-entrypoint
# ENTRYPOINT ["python", "main.py"]

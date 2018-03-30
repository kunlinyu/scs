TEX2PAGE=~/tex2page/my-tex2page
FILE=SCS-1.0.tex
LAFLAGS=-interaction=nonstopmode
all: pdf
pdf:
	xelatex -no-pdf $(LAFLAGS) $(FILE)
	xelatex -no-pdf $(LAFLAGS) $(FILE)
	xelatex $(LAFLAGS) $(FILE)
html:
	$(TEX2PAGE) $(FILE)
	$(TEX2PAGE) $(FILE)
	$(TEX2PAGE) $(FILE)
	sed -i 's/\[language=C++\]//g' *.html
	sed -i 's/\[numbers=none\]//g' *.html
clean:
	rm -rf *.aux *.xdv *.idx *.log *.out *.toc SCS-1.0.pdf

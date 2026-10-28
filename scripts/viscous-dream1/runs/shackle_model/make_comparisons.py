"""Place unwarped photo crops beside deterministic geometry renders."""
from pathlib import Path
import json
from PIL import Image, ImageDraw, ImageFont, ImageOps

RUN=Path(__file__).resolve().parent;ROOT=RUN.parents[1]
font_path='/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf'
font=ImageFont.truetype(font_path,27);small=ImageFont.truetype(font_path,21)
specs={
    289:{'photo_crop':(20,325,815,945),'model_crop':(35,140,970,925),
         'note':'Body only; near clear ear, far tapped ear. Visual tapped hole is a plain minor bore.'},
    296:{'photo_crop':(25,290,850,1030),'model_crop':(20,65,970,945),
         'note':'Pin seated; head rotated 90 deg about its axis to show the narrow tab edge as in the photo.'},
    288:{'photo_crop':(350,250,610,895),'model_crop':(330,75,700,915),
         'note':'Pin standing on its threaded tip. Thread geometry is intentionally a plain major cylinder.'},
}
for n,spec in specs.items():
    photo=Image.open(next((ROOT/'shackle/photos').glob(f'*m{n}.jpg'))).convert('RGB').crop(spec['photo_crop'])
    model=Image.open(RUN/f'render_m{n}.png').convert('RGB').crop(spec['model_crop'])
    canvas=Image.new('RGB',(1800,1110),'#f1f2f3');d=ImageDraw.Draw(canvas)
    d.rectangle((0,0,1800,86),fill='#202b38')
    d.text((30,24),f'm{n} — original photo crop',font=font,fill='white')
    d.text((930,24),'Parametric model — approximate viewpoint',font=font,fill='white')
    for im,x in [(photo,25),(model,925)]:
        fit=ImageOps.contain(im,(850,900),Image.Resampling.LANCZOS)
        canvas.paste(fit,(x+(850-fit.width)//2,100+(900-fit.height)//2))
    d.line((900,86,900,1010),fill='#bec4ca',width=2)
    d.text((30,1025),spec['note'],font=small,fill='#263548')
    d.text((30,1064),'Photo and model scaled independently; no perspective calibration. Dimensions and uncertainty: dimension_check.md.',font=small,fill='#536172')
    canvas.save(RUN/f'compare_m{n}.png')
(RUN/'comparison_views.json').write_text(json.dumps(specs,indent=2)+'\n')

# Quick index includes assembled/exploded and all requested photo comparisons.
cards=[]
for filename,title in [('assembled.png','Assembled — task frame'),('exploded.png','Exploded — insertion direction'),
                       ('compare_m289.png','m289 body'),('compare_m296.png','m296 assembled'),('compare_m288.png','m288 upright pin')]:
    im=Image.open(RUN/filename).convert('RGB');im.thumbnail((880,650),Image.Resampling.LANCZOS)
    card=Image.new('RGB',(900,710),'white');card.paste(im,((900-im.width)//2,50+(650-im.height)//2))
    ImageDraw.Draw(card).text((20,12),title,font=font,fill='#263548');cards.append(card)
index=Image.new('RGB',(1800,2130),'#e5e9ed')
for i,card in enumerate(cards):index.paste(card,((i%2)*900,(i//2)*710))
index.save(RUN/'review_contact_sheet.png')
print('Wrote three comparisons and review_contact_sheet.png')
